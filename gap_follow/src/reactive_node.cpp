#include <sstream>
#include <string>
#include <cmath>
#include <array>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define ARRAY_LENGTH 1080

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

    float max_value = 1.4;
    array<float, ARRAY_LENGTH> last_processed_ranges = {};

    flaot preprocess_lidar(float ranges_arr[])
    {   

	array<float, ARRAY_LENGTH> ranges = {};

	for (int i = 0; i < ARRAY_LENGTH; i++)
	{
	    ranges[i] = (ranges_arr[i] + last_processed_ranges[i]) / 2;
	    // 이전 데이터와의 평균으로 어느정도 보정 
	    last_processed_ranges[i] = ranges_arr[i];
	}
	    
        ranges = std::clamp(ranges, 0, max_value);
        return ranges;
	
    }

    int find_closest_point(float ranges_arr[])
    {   
	// 인덱스 반환
        return min_element(ranges_arr, ranges_arr + 1080) - ranges_arr.begin();
    }

    void process_bubble(float& ranges_arr[], int point_idx)
    {
        left, right = max_element(ranges_arr.begin(), ranges_arr.begin() + (point_idx - 100)), min_element(ranges_arr.begin() + (len(ranges)-1), ranges_arr.begin() + (point_idx + 99));
        ranges_arr[left: right+1] = [0] * (right - left + 1);
    }

    void find_max_gap(float ranges_arr[])
    {   

	vector<int> split_idx 
        copy_if(ranges_arr.begin(), ranges_arr.end(), back_inserter(split_idx), ranges_arr == 0.0);
	// 값이 0.0인 인덱스 색출(동적)
        sranges = ;
	// split_idx를 기준으로 gap 분할
        len_sranges = ;
	// gap의 길이를 정리한 배열
        max_idx = ;
	// 가장 길이가 긴 gap 선정
        if (max_idx == 0)
        {
	    start_i = 0;
	    // max_gap start_index
	    end_i = len_sranges[0]-1;
	    // max_gap_end_index
        }
        else
        {
	    start_i =np.sum(len_sranges[:max_idx]);
	    // max_gap start_index
	    end_i = start_i+len_sranges[max_idx]-1;
	    // max_gap_end_index
        }
        max_length_ranges = sranges[max_idx];
	// max_gap의 길이
        return start_i, end_i, max_length_ranges;
    }

    void find_best_point(float* ranges, int* indice)
    {   
	idx_list = np.where(ranges == np.max(ranges))[0];
        best_idx = start_i + idx_list[round(len(idx_list)/2)];
        return best_idx;
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
       
	array<float, ARRAY_LENGTH> ranges_arr = scan_msg->ranges;
	    
        ranges_arr = preprocess_lidar(ranges_arr);
	// 이전 배열값과의 평균
        int closest_point = find_closet_point(ranges_arr);
        // 가장 작은 배열 요소의 인덱스
        ranges_arr = process_bubble(ranges_arr, closest_point);
        // 가장 작은 배열 요소의 인덱스 주변의 배열 요소를 0으로 초기화
        max_gap_start, max_gep_end, max_gap_ranges = find_max_gap(ranges_arr);
        // max_gap을 찾는 과정
	max_point = find_base_point(max_gap_start, max_gap_end, max_gap_ranges);
        //
	
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
