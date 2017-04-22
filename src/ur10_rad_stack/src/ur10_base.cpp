
// System
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Messages
#include <geometry_msgs/Pose.h>

// MoveIt
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <geometric_shapes/solid_primitive_dims.h>

using namespace ros;
using namespace std;

std::string gripper_tool_frame = "ee_link";
moveit::planning_interface::MoveGroup* move_group_arm;
float exec_wait_ = 0.01f;
ros::Publisher display_publisher;
ros::Publisher pub_co;
ros::Publisher pub_aco;


moveit::core::RobotState CreateEmptyRobotState() {
  moveit::core::RobotState state(*move_group_arm->getCurrentState());
  // ROS_INFO_STREAM("state" << state);
  // state.setToDefaultValues();
  return state;
}

bool RobotStateFromPose(const geometry_msgs::Pose p,
                        moveit::core::RobotState& robot_sate) {
  // state(*move_group_arm->getCurrentState());
  ROS_INFO_STREAM("p:" << p);
  const robot_state::JointModelGroup* joint_model_group =
    robot_sate.getJointModelGroup(move_group_arm->getName());
  ROS_INFO_STREAM("move_group_arm->getName() - " << move_group_arm->getName());
  bool ok = robot_sate.setFromIK(joint_model_group, p, gripper_tool_frame);
  if (!ok) {
    ROS_ERROR_STREAM("Cannot get IK to go to " << p);
  }
  return ok;
}

bool Plan(moveit::core::RobotState start,
          moveit::core::RobotState end,
          moveit::planning_interface::MoveGroup::Plan& plan,
          geometry_msgs::Quaternion orient_constraint = geometry_msgs::Quaternion()) {

  move_group_arm->setStartState(start);
  move_group_arm->setJointValueTarget(end);
  bool success = move_group_arm->plan(plan);

  return success;
}


bool MoveTo(geometry_msgs::Pose p) {
  moveit::planning_interface::MoveGroup::Plan moveto_plan;

  moveit::core::RobotState moveto_robot_state = CreateEmptyRobotState();
  bool success = RobotStateFromPose(p, moveto_robot_state);
  if (!success) {
    ROS_WARN_STREAM("Cannot get robot pose for some of the needed poses.");
  } else {
    success &= Plan(*move_group_arm->getCurrentState(),
                    moveto_robot_state,
                    moveto_plan);
  }
  ROS_INFO_STREAM("[UR10_BASE] Planning 'MoveTo': " <<
                  ((success) ? "success" : "fail"));
  if (success) {
    ROS_INFO_STREAM("[UR10_BASE] Executing on the robot ...");

    // Execute on the arm
    // success &= move_group_arm->execute(moveto_plan);

    ROS_INFO("Visualizing plan");
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = moveto_plan.start_state_;
    display_trajectory.trajectory.push_back(moveto_plan.trajectory_);
    display_publisher.publish(display_trajectory);

    if (!success) {
      ROS_WARN("Failed execution on the robot of MoveTo.");
    }
    ros::WallDuration(exec_wait_).sleep();
  } else {
    ROS_INFO_STREAM("[UR10_BASE] Failed to find a plan for pose: " << p);
  }

  return success;
}

moveit_msgs::CollisionObject deleteObject(
  std::string object_id) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm->getPlanningFrame();
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = object_id;
  // Remove any previous occurrences in the world
  collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(collision_object);
  // ros::WallDuration(co_wait_).sleep();
  return collision_object;
}

void AddCollisionBox() {
  moveit_msgs::CollisionObject collision_object = deleteObject("top");
  collision_object.header.frame_id = "/world";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(
    geometric_shapes::SolidPrimitiveDimCount
    <shape_msgs::SolidPrimitive::BOX>::value);
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 2.54;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.5;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;

  // A pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_top_pose;
  table_top_pose.orientation.w = 1.0;
  table_top_pose.position.x =  0.0;
  table_top_pose.position.y =  1.0;
  table_top_pose.position.z =  -0.01;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_top_pose);
  collision_object.operation = collision_object.ADD;

  // pub_co.publish(collision_object);
  ROS_INFO("[PICKPLACEACTION] Adding an object into the world ...");
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.addCollisionObjects(collision_objects);
}

int main(int argc, char** argv) {
  std::cout << "staritng ... " << std::endl;
  ros::init(argc, argv, "ur10_base");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Initialize moveit group
  move_group_arm = new moveit::planning_interface::MoveGroup("manipulator");
  float max_planning_time_ = 10.0;
  int num_planning_attempts_ = 10;
  move_group_arm->setPlanningTime(max_planning_time_);  // Seconds
  move_group_arm->setNumPlanningAttempts(num_planning_attempts_);
  move_group_arm->setEndEffectorLink("ee_link");

  display_publisher =
    nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",
                                                 1, true);
  pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  // pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>(
  //             "attached_collision_object", 10);
  // ROS_INFO_STREAM(move_group_arm->getPlanningFrame());


  ROS_INFO_STREAM(move_group_arm->getPlanningFrame());
  ROS_INFO_STREAM("pose: " << move_group_arm->getCurrentPose());
  ROS_INFO_STREAM("eff: " << move_group_arm->getEndEffector());
  ROS_INFO_STREAM("link: " << move_group_arm->getEndEffectorLink());

  ros::WallDuration(2.0).sleep();
  AddCollisionBox();


  geometry_msgs::Pose p = move_group_arm->getCurrentPose().pose;
  p.position.z -= 0.05;
  // p.position.x = 0;
  // p.position.y = 0;
  // p.position.z = 0.01;

  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.0;
  p.orientation.w = 1.0;

  // p.position.x = -0.28;
  // p.position.y = 0.26;
  // p.position.z = 0.96; //1.0;

  // p.orientation.x = 0.263;
  // p.orientation.y = -0.408;
  // p.orientation.z = -0.549;
  // p.orientation.w = 0.680;

  // move_group_arm->setPositionTarget(-0.28, 0.26, 0.95, "ee_link");
  // moveit::planning_interface::MoveGroup::Plan my_plan;
  // bool success = move_group_arm->plan(my_plan);
  // if (success)
  //   ROS_INFO("@@ok!");

  // move_group_arm->execute(my_plan);

  ROS_INFO("STARTINGGG");
  MoveTo(p);
  ROS_INFO("SENT!");
  ROS_INFO("llooop");


  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
