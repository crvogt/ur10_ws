#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>

class UR10Collect{
private:
	geometry_msgs::Pose target_pose;
	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::NodeHandle *nh_;

public:
	void run(void)
};

UR10Collect::UR10Collect(ros::NodeHandle *nh) : nh_(nh){

}

void UR10Collect::run(void){

    target_pose.position.x = 0.0;
    target_pose.position.y = 0.0;
    target_pose.position.z = -0.5;
    target_pose.orientation.x = 0;
    target_pose.orientation.y = -0.707;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = 0.707;
    
    group.setPoseTarget(target_pose);
    ros::Duration(5.0); // UNCOMMENT THIS
}




int main(int argc, char **argv){

	ros::init(argc, argv, "ur10_endpoint_command_node");
    ros::NodeHandle nh;
    UR10Collect ur10Instance(&nh);
    //ros::Subscriber subImage = nh.subscribe("/usb_cam/image_raw", 1, &PR2Collect::getImageCallback, &pr2Instance);

    //Might want this to go in the while loop...
    ur10Instance.run();

    ros::Rate rate(200.0);

    while(ros::ok()){
      ros::spinOnce();
      rate.sleep();
    }

	return 1;
}