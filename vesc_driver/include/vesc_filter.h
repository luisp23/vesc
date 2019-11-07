#ifndef VESC_FILTER_H_
#define VESC_FILTER_H_

#include <ros/ros.h>
#include <math.h>      
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

namespace vesc_filter{ 
	class VescFilter{ 

		public: 
			VescFilter(ros::NodeHandle nh, ros::NodeHandle private_nh); 

		private: 
			int filter_size; 
			float filter_rate; 
			float publisher_rate; 
			float max_vel;
			float duty_cycle_max;
			float duty_cycle_min; 
			float steering_angle_max;
			float steering_angle_min; 

			geometry_msgs::Twist twist_input; 
			std::vector<float> linear_vel;
			std::vector<float> steering_angle;
			float avg_linear_vel; 
			float avg_steering_angle; 

			ros::Publisher duty_cycle_pub;
			ros::Publisher steering_angle_pub;
			ros::Publisher break_signal;

			ros::Subscriber twist_sub;

			ros::Timer filter_timer_;
			ros::Timer publisher_timer; 

			void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg); 
			void filterTimerCallback(const ros::TimerEvent& event); 
			void publisherTimerCallback(const ros::TimerEvent& event);
	};
}
#endif 