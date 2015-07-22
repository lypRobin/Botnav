#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <termios.h>
#include <unistd.h>

#define MIN(a,b) ((a) >= (b) ? (b):(a))
#define MAX(a,b) ((a) >= (b) ? (a):(b))

const float speed_factor[][2] = {{1.1, 1.1}, 
						   		 {0.9, 0.9}, 
						   		 {1.1, 1.0}, 
						   		 {0.9, 1.0}, 
						   		 {1.0, 1.1}, 
						   		 {1.0, 0.9}};

/******************************
*  9 directions for direction_factor
*
*   \   |   /      0  1  2
*    \  |  /   
*  ---  o  ----    3  4  5
*    /  |  \
*   /   |   \      6  7  8
*
*/
const int direction_factor[][2] = {{1, 1},   // forward left
								   {1, 0},  // forward 
								   {1, -1},   // forward right 
								   {0, 1},  // turn left
								   {0, -1},   // right
								   {-1, -1},  // back left
								   {-1, 0},  // back 
								   {-1, 1}}; // back right
int count = 0; // speed decrease smoothly
// initialise
float x = 0.0;
float arc = 0.0;
float target_speed = 0.0;
float control_speed = 0.0;
float target_turn = 0.0;
float control_turn = 0.0;

class BaseData
{
private:
	ros::NodeHandle cmd_nh_;
	ros::NodeHandle odom_nh_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber odom_sub_;
	nav_msgs::Odometry odom_data_;
	bool quit_process_;
	float speed_;
	float turn_;

public:
	BaseData()
	{
		cmd_vel_pub_ = cmd_nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
		odom_sub_ = odom_nh_.subscribe("/odom", 1000, &BaseData::get_odometry_callback, this);
		quit_process_ = false;
		speed_ = 0.2;
		turn_ = 1.0;
	}

	~BaseData()
	{

	}

public:
	void print_cmd()
	{
		std::cout << "Keyboatd Control Turtlebot.\n "
		"==============================\n"
		"Direction Control:\n"
		"q   w   e\n"
		"a       d\n"
		"z   x   c\n" 
		"s or Space:  stop moving!\n"
		"==============================\n"
		"Speed Control:\n"
		"u/i increase / decrease speed by 10 persent.\n"
		"                                  \n"
		"ESC or Ctrl+C  exit\n" << std::endl;
	}

	char get_key()
	{
		char key = 0;
		struct termios io = {0};
		if(tcgetattr(fileno(stdin), &io) < 0)
			ROS_INFO("ERROR: tcgetattr");
		io.c_lflag &= ~ICANON;
		io.c_lflag &= ~ECHO;
		io.c_cc[VMIN] = 1;
		io.c_cc[VTIME] = 0;
		if (tcsetattr(fileno(stdin), TCSANOW, &io) < 0)
			ROS_INFO("ERROR:tcsetattr ICANON");
		if(read(fileno(stdin), &key, 1) < 0)
			ROS_INFO("ERROR:read");
		io.c_lflag |= ICANON;
		io.c_lflag |= ECHO;
		if(tcsetattr(fileno(stdin), TCSADRAIN, &io) < 0)
			ROS_INFO("ERROR: tcgetattr ~ICANON");

		return key;
	}

	bool is_valid_key(const char k)
	{
		if(k != 'w' && k != 'a' && k != 's' && k != 'd' && k != 'x'
			&& k != 'q' && k != 'e' && k != 'z' && k != 'c' && k != 'j' 
			&& k!= 'k' && k!= ' ')
			return false;

		return true;
	}

	void keyboard_control()
	{
		// print_cmd();

		geometry_msgs::Twist base_cmd;
		char k = get_key();

		if(!is_valid_key(k)){
			std::cout << "Unknown command: " << k << "\n" << std::endl;
			print_cmd();
		}

		//speed control
		if(k == 'u'){
			speed_ = speed_ * speed_factor[0][0];
			turn_ = speed_ * speed_factor[0][1];
			count = 0;
		}
		else if(k == 'i'){
			speed_ = speed_ * speed_factor[1][0];
			turn_ = speed_ * speed_factor[1][1];
			count = 0;
		}
		// direction control
		else if(k == ' ' || k == 's'){
 			x = 0.0;
 			arc = 0.0;
 			control_speed = 0.0;
 			control_turn = 0.0;
		}
		else if(k == 'w')
		{
			x = direction_factor[1][0];
			arc = direction_factor[1][1];
			count = 0;
		}
		else if(k == 'a')
		{
			x = direction_factor[3][0];
			arc = direction_factor[3][1];
			count = 0;
		}
		else if(k == 'd')
		{
			x = direction_factor[4][0];
			arc = direction_factor[4][1];
			count = 0;
		}
		else if(k == 'q')
		{
			x = direction_factor[0][0];
			arc = direction_factor[0][1];
			count = 0;
		}
		else if(k == 'e')
		{
			x = direction_factor[2][0];
			arc = direction_factor[2][1];
			count = 0;
		}
		else if(k == 'z')
		{
			x = direction_factor[5][0];
			arc = direction_factor[5][1];
			count = 0;
		}
		else if(k == 'c')
		{
			x = direction_factor[7][0];
			arc = direction_factor[7][1];
			count = 0;
		}
		else if(k == 'x'){
			x = direction_factor[6][0];
			arc = direction_factor[6][1];
			count = 0;
			
		}
		else{
			// if(count > 4){
			// 	x = 0.0;
			// 	arc = 0.0;
			// }
			if(k == '\x1B')
				quit_process_ = true;//break;
			count++;
		}

		target_speed = speed_ * x;
		target_turn = turn_ * arc;
		if(target_speed > control_speed)
			control_speed = MIN(target_speed, control_speed + 0.02);
		else if(target_speed < control_speed)
			control_speed = MAX(target_speed, control_speed - 0.02);
		else 
			control_speed = target_speed;

		if(target_turn > control_turn)
			control_turn = MIN(target_turn, control_turn + 0.1);
		else if(target_turn < control_turn)
			control_turn = MAX(target_turn, control_turn - 0.1);
		else 
			control_turn = target_turn;

		base_cmd.linear.x = control_speed;
		base_cmd.linear.y = 0;
		base_cmd.linear.z = 0;

		base_cmd.angular.x = 0;
		base_cmd.angular.y = 0;
		base_cmd.angular.z = control_turn;

		cmd_vel_pub_.publish(base_cmd);

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
		while(ros::ok()){
			if(quit_process_)
				break;

			keyboard_control();
			// get_odometry_data();
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




