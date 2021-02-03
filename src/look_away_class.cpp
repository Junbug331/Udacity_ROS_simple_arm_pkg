#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

#include "simple_arm/GoToPosition.h"

class LookAway
{
public:
	LookAway()
	{
		joints_last_position = {.0, .0};
		moving_state = false;
		client_ = nh_.serviceClient<simple_arm::GoToPosition>("arm_mover/safe_move");
		sub1_ = nh_.subscribe("/simple_arm/joint_state", 10, &LookAway::joint_state_callback, this);
		sub2_ = nh_.subscribe("rgb_camera/image_raw", 10, &LookAway::look_away_callback, this);
	}
	
	void move_arm_center()
	{
		ROS_INFO_STREAM("Moving arm to the center");

		simple_arm::GoToPosition srv;
		srv.request.joint_1 = 1.57;
		srv.request.joint_2 = 1.57;
		
		if (!client_.call(srv))
			ROS_ERROR("Failed to call service /arm_mover/safe_move");
	}

	void joint_state_callback(const sensor_msgs::JointState js)
	{
		std::vector<double> joints_current_position = js.position;
		
		double tolerance = 0.0005;

		if (fabs(joints_current_position[0] - joints_last_position[0]) < tolerance && fabs(joints_current_position[1] - joints_last_position[1]) < tolerance)
			moving_state = false;
		else
		{
			moving_state = true;
			joints_last_position = joints_current_position;
		}
	}


	void look_away_callback(const sensor_msgs::Image img)
	{
		bool uniform_image = true;

		for (int i = 0; i < img.height * img.step; i++)
		{
			if (img.data[i] - img.data[0] != 0)
			{
				uniform_image = false;
				break;
			}		
		}
		
		if (uniform_image == true && moving_state == false)
			move_arm_center();	
	}
	
private:
	ros::NodeHandle nh_;
	ros::ServiceClient client_;
	ros::Subscriber sub1_;
	ros::Subscriber sub2_;

	std::vector<double> joints_last_position;
	bool moving_state;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "look_away");

    LookAway lookAway;

    ros::spin();

    return 0;
}
