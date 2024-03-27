#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std;

const string &lidarscan_topic = "/scan";
const string &drive_topic = "/drive";

class ReactiveFollowGap : public rclcpp::Node {

public:
    
    ReactiveFollowGap() : Node("reactive_node")
    {
    
		    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));
        
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10); 
    
    }

private:

		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    
    float last_processed_ranges[1080] = { 0, };

    void preprocess_lidar(float* ranges)
    {   
        float max_value = 1.4;
        proc_ranges = ranges;
        ranges = (proc_ranges + last_processed_ranges) / 2;
        last_processed_ranges = proc_ranges;
        
        ranges = std::clamp(ranges, 0, max_value);
        return ranges;
    }

		int find_closest_point(float* ranges)
    {   
        return *std::min_element(ranges.begin(), ranges.end());
    }

		
    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        return;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }

		float get_velocity(float steering_angle)
    {
		
        if (abs(steering_angle) >= deg2rad(0) && abs(steering_angle) < deg2rad(10):
				{
	          return 3.0;
				}
		    
				elif (abs(steering_angle) >= deg2rad(10) && abs(steering_angle) < deg2rad(20):
				{
				    return 1.0;
				}
				
		    else
		    {
						return 0.5;
		    }
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        
        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;
       
        const vector<float> &ranges = scan_msg->ranges;
        
        proc_ranges = preprocess_lidar(ranges);
        closest_point = find_closet_point(proc_ranges);
        
        bubble_processed_ranges = process_bubble(proc_ranges, closest_point);
        
        max_gap_start, max_gep_end, max_gap_ranges = find_max_gap(bubble_processed_ranges);
        max_point = find_base_point(max_gap_start, max_gap_end, max_gap_ranges);
        
        steering_angle = (max_point * angle_increment) + angle_min;
        
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = get_speed(steering_angle);
        drive_publisher_->publish(drive_msg);
    
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
