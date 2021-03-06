#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "simple_arm/GoToPosition.h"

class ArmMover
{
public:
    ArmMover()
    {
        joint1_pub = nh_.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
        joint2_pub = nh_.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);
        service_ = nh_.advertiseService("/arm_mover/safe_move", &ArmMover::handle_safe_move_request, this);
        ROS_INFO("Ready to send joint commands");
    }

    std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2)
    {
        float clamped_j1 = requested_j1;
        float clamped_j2 = requested_j2;

        float min_j1, max_j1, min_j2, max_j2;

        std::string node_name = ros::this_node::getName();

        nh_.getParam(node_name + "/min_joint_1_angle", min_j1);
		nh_.getParam(node_name + "/max_joint_1_angle", max_j1);
		nh_.getParam(node_name + "/min_joint_2_angle", min_j2);
		nh_.getParam(node_name + "/max_joint_2_angle", max_j2);
		
		if (requested_j1 < min_j1 || requested_j1 > max_j1)
		{
			clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
			ROS_WARN("j1 is out of bounds, valid range(%1.2f, %1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
		}

		if (requested_j2 < min_j2 || requested_j2 > max_j2)
		{
			clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2)		
			ROS_WARN("j2 is out of bounds, valid range(%1.2f, %1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
		}
		
		std::vector<float> clamped_data = {clamped_j1, clamped_j2};
		return clamped_data;
    }

	bool handle_safe_move_request(simple_arm::GoToPosition::Request &req, simple_arm::GoToPosition::Response &res)
	{
		ROS_INFO("GoToPositionRequest received = j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);
		std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);
		
		std_msgs::Float64 joint1_angle, joint2_angle;
		joint1_angle.data = joints_angles[0];
		joint2_angle.data = joints_angles[1];

		joint1_pub.publish(joint1_angle);
		joint2_pub.publish(joint2_angle);
	
		ros::Duration(3).sleep();

		res.msg_feedback = "Joint angle set - j1: " + std::to_string(joints_angles[0]) + " ,j2: " + std::to_string(joint_angles[1]);
		ROS_INFO_STREAM(res.msg_feedback);

		return true;
	}


private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    ros::Publisher joint1_pub;
    ros::Publisher joint2_pub;
};

int main(int argc, char** argv)
{
	ros::inint(argc, argv, "arm_mover");

	ArmMover armMover;

	ros::spin();

    return 0;
}
