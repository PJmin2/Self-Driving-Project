#include <sstream>
#include <string>
#include <ctime>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std;

const string &lidarscan_topic = "/scan";
const string &drive_topic = "/drive";
const string &initialpose_topic = "/initialpose_topic";

class WallFollow(float kp, float ki, float kd) : public rclcpp::Node {

public:

    WallFollow() : Node("wall_follow_node")
    {
    
		    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
        
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10); 
        
        initialpose_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(initialpose_topic, 10); 
        
    }

private:

		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr initialpose_publisher_;
	
		ackermann_msgs::msg::AckermannDriveStamped drive_msg;
		
		float kp = kp;
		float ki = ki;
		float kd = kd;
		    
    float prev_error = 0.0;
    float error = 0.0;
    float integral = 0.0;
    
    float lookahead_dist = 1.0;
    float current_reading_time = time();

    float get_range(float* range_data, float angle)
    {
				
				int index = static_cast <int> (std::floor((angle - angle_min) / angle_increment));
        return range_data[index]
    
    }

    float get_error(float* range_data, float dist)
    {
        
        float angle_b = M_PI / 2.0;
        float angle_a = M_PI / 4.0;
        
        while (!isnan(get_range(range_data, angle_a)) || (angle_a == angle_b))
        {
            printf('skipping a');
            angle_a += 0.02;
        }
        
        while (!isnan(get_range(range_data, angle_b)) || (angle_a == angle_b))
        {
            printf('skipping b');
            angle_b += 0.02;
        }

        float a = get_range(range_data, angle_a);
        float b = get_range(range_data, angle_b);
        float theta = angle_b - angle_a;
        
        float alpha = atan2((a * cosf(theta) - b), (a * sinf(theta));
        
        float curr_dist = b * cosf(alpha);
        
        float lookahead_dist = curr_dist + lookahead_dist * sinf(alpha);
        
        return dist - lookahead_dist;
    }
	
		float get_velocity(float angle)
		{
		
		    if (angle < M_PI/18)
		    {
		        return 1.5
		    }
		    
		    if (angle < M_PI/9)
		    {
		        return 1.0
		    }
		    
		    return 0.5
		    
		}
		
    void pid_control(float error)
    {
        
        float angle = 0.0;
        
        float previous_reading_time = current_reading_time;
        current_reading_time = time();
        float dt = current_reading_time - previous_reading_time;
        
        integral = prev_error * dt;
        float derivative = (error - prev_error) / dt;
        
        angle = -(kp * error + ki * integral + kd * derivative);
        
        prev_error = error;
        
        if (abs(error) < 0.1)
        {
	          angle = 0.0;
        }
        
        float velocity = get_velocity(abs(angle));
        
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = angle;
        drive_publisher_->publish(drive_msg);
        
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
	    
		    float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;
       
        const vector<float> &ranges = scan_msg->ranges;
        				
        float error = get_error(&ranges, 1.0);
        pid_control(error);
       
    }

};

float getUserInput(const std::string& prompt)
{
    std::cout << prompt;
    std::string input;
    std::cin >> input;
    
    return std::stof(input)
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    float kp = getUserInput("kp: ");
	  float kd = getUserInput("kd: ");
	  float ki = getUserInput("ki: ");
    rclcpp::spin(std::make_shared<WallFollow>(kp, ki, kd));
    rclcpp::shutdown();
    return 0;
}
