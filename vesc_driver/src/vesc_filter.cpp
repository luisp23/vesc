#include "vesc_filter.h"

namespace vesc_filter{ 
	VescFilter::VescFilter(ros::NodeHandle nh, ros::NodeHandle private_nh): avg_linear_vel(0.0){
		
		private_nh.getParam("filter_size", filter_size);
	    linear_vel.resize(filter_size); 
	    private_nh.getParam("filter_rate", filter_rate);
	    private_nh.getParam("publisher_rate", publisher_rate);
	    private_nh.getParam("max_vel", max_vel); 
	    private_nh.getParam("duty_cycle_max", duty_cycle_max);  
	    private_nh.getParam("duty_cycle_min", duty_cycle_min);
	    private_nh.getParam("steering_angle_max", steering_angle_max);  
	    private_nh.getParam("steering_angle_min", steering_angle_min);

	    duty_cycle_pub = nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 1);
	    steering_angle_pub = nh.advertise<std_msgs::Float64>("commands/servo/position", 1);
	    break_signal = nh.advertise<std_msgs::Float64>("commands/motor/brake", 1);

	   	twist_sub = nh.subscribe("remote", 1, &VescFilter::cmdVelCallback, this);

	    filter_timer_ = nh.createTimer(ros::Duration(1.0/filter_rate), &VescFilter::filterTimerCallback, this);
	    publisher_timer = nh.createTimer(ros::Duration(1.0/publisher_rate), &VescFilter::publisherTimerCallback, this);
	}

	void VescFilter::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
		twist_input.linear.x = msg->linear.x; 
	    twist_input.angular.z = msg->angular.z/2.0 + 0.5;
	}

	void VescFilter::filterTimerCallback(const ros::TimerEvent& event){
		std::rotate(linear_vel.begin(),linear_vel.begin()+1,linear_vel.end());
		linear_vel[filter_size-1] = twist_input.linear.x;
		
		float sum_linear_vel = 0; 
		for(int i = 0; i < filter_size; i++){     
			sum_linear_vel += linear_vel[i];
		}

		avg_linear_vel = sum_linear_vel/filter_size;
	}

	void VescFilter::publisherTimerCallback(const ros::TimerEvent& event){
		std_msgs::Float64 duty_cycle_msg;
	    float duty_cycle = avg_linear_vel/max_vel; 
	    if(fabs(duty_cycle) >= duty_cycle_max){
	        duty_cycle_msg.data = duty_cycle_max;
	    }else if(fabs(duty_cycle) >= duty_cycle_min){
	    	duty_cycle_msg.data = duty_cycle; 
	    }else{
	        duty_cycle_msg.data = 0.0; 
	    } 	    
	    
		std_msgs::Float64 steering_angle_msg;	    
		float angle = twist_input.angular.z;
		
		if(fabs(angle) >= steering_angle_max){
			steering_angle_msg.data = steering_angle_max;
		}else if(fabs(angle-0.5) >= steering_angle_min){
			steering_angle_msg.data = angle; 
		}else{
			steering_angle_msg.data = 0.5; 
		} 
			
	    duty_cycle_pub.publish(duty_cycle_msg);
	    steering_angle_pub.publish(steering_angle_msg);
	}
}
