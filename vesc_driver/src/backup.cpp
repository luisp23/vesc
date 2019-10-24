#include <ros/ros.h>
#include <math.h>      
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

std::vector<float> linear_vel(50);
std::vector<float> angular_vel(50);
float avg_linear_vel = 0; 
float avg_angular_vel = 0; 
int counter = 0; 
float max_vel = 0; 
float length = 0; 

float odom_linear_velocity = 0;
geometry_msgs::Twist::ConstPtr&


// Running average filter  	
void filterCallback(const geometry_msgs::Twist::ConstPtr& msg){
    if(counter == 49){
        for(int i = 0; i < 50 i++){
            avg_linear_vel += linear_vel[i];
            avg_angular_vel += angular_vel[i];
        }
        avg_linear_vel /= 50;
        avg_angular_vel /= 50;
        counter = 0;
	}else{ 
        linear_vel[counter]= msg.linear.x;
        angular_vel[counter]= msg.angular.x;
  }
} 

void odomCallback(onst nav_msgs::Odometry::ConstPtr& msg){
    odom_linear_velocity = msg.twist.linear.x; 

}

void timerCallback(const ros::TimerEvent& event){
    std_msgs::Float64 duty_cycle_msg;
    duty_cycle_msg->data = avg_linear_vel/max_vel;

    float radius = odom_linear_velocity/avg_angular_vel;    // use the current velocity of the vehicle to estimate the desired turning radius from desired angular_vel
    float angle = atan2(length/radius); 

    std_msgs::Float64 steering_angle_msg;
    steering_angle_msg->data = angle;
    duty_cycle_pub.publish(duty_cycle);
    steering_angle_pub.publish(angle) 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "vesc_filter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Publisher duty_cycle_pub = nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 10);
    ros::Publisher steering_angle_pub = nh.advertise<std_msgs::Float64>("commands/motor/position", 10);
    ros::Publisher break_signal = nh.advertise<std_msgs::Float64>("commands/motor/brake", 10);
    
    ros::Subscriber odom_sub =  nh.subscribe("odom", 10, &odomCallback, this);
    ros::Subscriber twist_sub = nh.subscribe("cmd_vel", 10, &filterCallback, this);
    ros::Timer timer_ = nh.createTimer(ros::Duration(1.0/50.0), timerCallback, this);

    while(ros::ok()){
        ros::spin();
    }
    return 0;
}
