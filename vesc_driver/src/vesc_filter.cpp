#include <ros/ros.h>
#include <math.h>      
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

int filter_size; 
float filter_rate; 
float publisher_rate; 
float max_vel;
float car_length;

std::vector<float> linear_vel(50);
std::vector<float> angular_vel(50);

float avg_linear_vel = 0; 
float avg_angular_vel = 0; 
int counter = 0; 
 

float odom_linear_velocity = 0;
geometry_msgs::Twist twist_input; 

ros::Publisher duty_cycle_pub;
ros::Publisher steering_angle_pub;
ros::Publisher break_signal;

// Running average filter  	
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    twist_input.linear.x = msg->linear.x; 
    twist_input.linear.z = msg->angular.z;
} 

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odom_linear_velocity = msg->twist.twist.linear.x; 
}

void filterTimerCallback(const ros::TimerEvent& event){
    if(counter == (filter_size-1)){
        for(int i = 0; i < filter_size; i++){
            avg_linear_vel += linear_vel[i];
            avg_angular_vel += angular_vel[i];
        }
        avg_linear_vel /= filter_size;
        avg_angular_vel /= filter_size;
        counter = 0;
        linear_vel.clear(); 
        angular_vel.clear(); 
    }else{ 
        linear_vel[counter]= twist_input.linear.x;
        angular_vel[counter]= twist_input.angular.z;
        counter++; 
    }
}


void publisherTimerCallback(const ros::TimerEvent& event){
    std_msgs::Float64 duty_cycle_msg;
    duty_cycle_msg.data = avg_linear_vel/max_vel;

    float radius = avg_linear_vel/avg_angular_vel;    // use the current velocity of the vehicle to estimate the desired turning radius from desired angular_vel
    float angle = atan2(car_length,radius); 
    
    std::cout << "Radius: " << radius << " Length: " << car_length << "Angle" << angle << std::endl;

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
    private_nh.getParam("filter_rate", filter_rate);
    private_nh.getParam("publisher_rate", publisher_rate);
    private_nh.getParam("max_vel", max_vel); 
    private_nh.getParam("car_length", car_length); 

    duty_cycle_pub = nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 10);
    steering_angle_pub = nh.advertise<std_msgs::Float64>("commands/motor/position", 10);
    break_signal = nh.advertise<std_msgs::Float64>("commands/motor/brake", 10);
    
    ros::Subscriber odom_sub =  nh.subscribe("odom", 10, &odomCallback);
    ros::Subscriber twist_sub = nh.subscribe("remote", 10, &cmdVelCallback);
    
    ros::Timer filter_timer_ = nh.createTimer(ros::Duration(1.0/filter_rate), filterTimerCallback);
    ros::Timer publisher_timer = nh.createTimer(ros::Duration(1.0/publisher_rate), publisherTimerCallback);

    while(ros::ok()){
        ros::spin();
    }
    return 0;
}
