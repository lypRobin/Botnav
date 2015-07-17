#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class BaseData
{
private:
	ros::NodeHandle cmd_nh_;
	ros::NodeHandle odom_nh_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber odom_sub_;
	nav_msgs::Odometry odom_data_;
	// geometry_msgs::PoseWithCovarianceStamped odom_data_;

public:
	BaseData()
	{
		cmd_vel_pub_ = cmd_nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
		odom_sub_ = odom_nh_.subscribe("/odom", 1000, &BaseData::get_odometry_callback, this);
	}

	~BaseData()
	{

	}

public:
	void print_cmd()
	{
		std::cout << "Keyboatd Control Turtlebot.\n "
		"==============================\n"
		"w   move forward\n"
		"s   move backward\n"
		"a   turn left\n"
		"d   turn right\n" 
		"q   quit\n" << std::endl;
	}

	bool keyboard_control()
	{
		print_cmd();

		geometry_msgs::Twist base_cmd;
		char cmd[50];

		std::cin.getline(cmd, 50);
		if(cmd[0] != 'w' && cmd[0] != 'a' && cmd[0] != 's' && cmd[0] != 'd' && cmd[0] != 'q')
		{
			std::cout << "Unknown command: " << cmd << "\n" << std::endl;
			print_cmd();
			return false;
		}

		// initialise
		base_cmd.linear.x = 0;
		base_cmd.linear.y = 0;
		base_cmd.angular.z = 0;

		// move forward
		if(cmd[0] == 'w')
		{
			base_cmd.linear.x = 0.1;
		}
		else if(cmd[0] == 's')
		{
			base_cmd.linear.x = -0.1;
		}
		else if(cmd[0] == 'a')
		{
			base_cmd.angular.z = 0.75;
			base_cmd.linear.x = 0.25;
		}
		else if(cmd[0] == 'd')
		{
			base_cmd.angular.z = -0.75;
			base_cmd.linear.x = 0.25;
		}
		else if(cmd[0] == 'q')
			;//break;

		cmd_vel_pub_.publish(base_cmd);

		return true;

	}

	void get_odometry_callback(const nav_msgs::Odometry &data)
	{
		odom_data_ = data;
		float pos_x = odom_data_.pose.pose.position.x;
		float pos_y = odom_data_.pose.pose.position.y;
		float pos_z = odom_data_.pose.pose.position.z;

		float ori_x = odom_data_.pose.pose.orientation.x;
		float ori_y = odom_data_.pose.pose.orientation.y;
		float ori_z = odom_data_.pose.pose.orientation.z;
		float ori_w = odom_data_.pose.pose.orientation.w;

		std::cout << "================================\n"
			"Odometry data from turtlebot:\n"
			"-----------------\n"
			"Position:\n"
			"x:" << pos_x << "\n"
			"y:" << pos_y << "\n"
			"z:" << pos_z << "\n"
			"-----------------\n"
			"Orientation:\n"
			"x:" << ori_x << "\n"
			"y:" << ori_y << "\n"
			"z:" << ori_z << "\n"
			"w:" << ori_w << "\n"
			<< std::endl;
	}

	void get_odometry_data()
	{
		ros::Rate r(1);
		ros::spinOnce();
		r.sleep();	
	}

	void run()
	{
		while(ros::ok())
		{
			keyboard_control();
			get_odometry_data();
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "get_base_data");
	//ros::NodeHandle nh;
	BaseData bd;
	bd.run();

	return 0;
}




