#include <ros/ros.h>
#include <math.h>      
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

int filter_size; 
float filter_rate; 
float publisher_rate; 
float max_vel;
float duty_cycle_max;
float duty_cycle_min; 

std::vector<float> linear_vel;
std::vector<float> angular_vel;

float avg_linear_vel = 0; 
float avg_angular_vel = 0; 
 

geometry_msgs::Twist twist_input; 

ros::Publisher duty_cycle_pub;
ros::Publisher steering_angle_pub;
ros::Publisher break_signal;

// Running average filter  	
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    twist_input.linear.x = msg->linear.x; 
    twist_input.angular.z = msg->angular.z;
} 

void filterTimerCallback(const ros::TimerEvent& event){

    std::rotate(linear_vel.begin(),linear_vel.begin()+1,linear_vel.end());
    std::rotate(angular_vel.begin(),angular_vel.begin()+1,angular_vel.end());
    linear_vel[filter_size-1] = twist_input.linear.x;
    angular_vel[filter_size-1] = twist_input.angular.z;
  
    float sum_linear_vel = 0; 
    float sum_angular_vel  = 0;    
    for(int i = 0; i < filter_size; i++){     
       sum_linear_vel += linear_vel[i];
       sum_angular_vel += angular_vel[i];
    }
    avg_linear_vel = sum_linear_vel/filter_size;
    avg_angular_vel= sum_angular_vel/filter_size;
}

void publisherTimerCallback(const ros::TimerEvent& event){
    std_msgs::Float64 duty_cycle_msg;
    float duty_cycle = avg_linear_vel/max_vel; 
    if(fabs(duty_cycle) >= duty_cycle_max){
        duty_cycle_msg.data = duty_cycle_max;
    }else if(fabs(duty_cycle) >= duty_cycle_min){
    	duty_cycle_msg.data = duty_cycle; 
    }else{
        duty_cycle_msg.data = 0.0; 
    } 	    
    
    
    float angle = 0; 
    std_msgs::Float64 steering_angle_msg;
    steering_angle_msg.data = angle;

    duty_cycle_pub.publish(duty_cycle_msg);
    steering_angle_pub.publish(steering_angle_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "vesc_filter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("filter_size", filter_size);
    linear_vel.resize(filter_size); 
    angular_vel.resize(filter_size); 
    private_nh.getParam("filter_rate", filter_rate);
    private_nh.getParam("publisher_rate", publisher_rate);
    private_nh.getParam("max_vel", max_vel); 
    private_nh.getParam("duty_cycle_max", duty_cycle_max);  
    private_nh.getParam("duty_cycle_min", duty_cycle_min);

    duty_cycle_pub = nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 1);
    steering_angle_pub = nh.advertise<std_msgs::Float64>("commands/motor/position", 1);
    break_signal = nh.advertise<std_msgs::Float64>("commands/motor/brake", 1);
    
    ros::Subscriber twist_sub = nh.subscribe("remote", 1, &cmdVelCallback);
    
    ros::Timer filter_timer_ = nh.createTimer(ros::Duration(1.0/filter_rate), filterTimerCallback);
    ros::Timer publisher_timer = nh.createTimer(ros::Duration(1.0/publisher_rate), publisherTimerCallback);

    while(ros::ok()){
        ros::spin();
    }
    return 0;
}
