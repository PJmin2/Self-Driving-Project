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
    vector<float> last_processed_ranges;

    vector<flaot> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
    {   

	vector<float> filtered_ranges;

	for (int i = 0; i < ARRAY_LENGTH; i++)
	{
		
	    if (isnan(scan_msg->ranges[i]))
	    {
	        filtered_ranges.push_back(0.0);
	    }
	    else if (scan_msg->ranges[i] > max_value || isinf(scan_msg->ranges[i]))
	    {    
		filtered_ranges.push_back(max_value);
	    }
	    else
	    {
	        filtered_ranges.push_back(scan_msg->ranges[i]);
	    }
		
	}

	for (int i = 0; i < ARRAY_LENGTH; i++)
	{
		
	    float before_ranges = filtered_ranges[i];
	    filtered_ranges[i] = (filtered_ranges[i] + last_processed_ranges[i]) / 2;
	    // 이전 데이터와의 평균으로 어느정도 보정 
	    last_processed_ranges[i] = before_ranges;
	}
	
	return filtered_ranges;
	
    }
    
    int find_closest_point(vector<float>& ranges_vec)
    {   
	// 인덱스 반환
        return min_element(ranges_vec.begin(), ranges_vec.end()) - ranges_vec.begin();
    }

    void process_bubble(vecotr<float>* ranges_vec, int closest_index)
    {
	// ranges_vec 벡터를 주소로 받아와서 원본 수정
	// 가장 작은 거리 + 버블 반지름을 기준으로 그보다 작은경우 0으로 초기화 
	const float center_point_distance = ranges_vec->at(closest_index);
	ranges_vec->at(closest_index_idx) = 0.0;

	int current_index = closest_index;
        for (int i = 0; i < ARRAY_LENGTH; i++)
	{
	    if (ranges_vec->at(i) < canter_point_distance + bubble_radius)
	    {
	        ranges_vec->at(i) = 0.0;
	    }
	}
    }

    pair<int, int> find_max_gap(vector<float>& ranges_vec)
    {   
	int max_start = 0;
	int max_size = 0;
	int max_before = 0;

	int current_index = 0;
	//uint_8로 변경하기

	// 0.0이 나오면 그동안의 길이를 저장하고 current_index 업데이트
	while (current_index < ARRAY_LENGTH)
	{
    	    max_szie = 0;
	    while (ranges_vec[current_index] != 0.0)
	    {
        	current_index++;
		max_size++;
	    }
            
	    if (max_size > max_before)
	    {
	        max_before = max_size;
		max_start = current_index - max_size;
	    }

	    current_index++;
	    
	}
	    
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


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg) 
    {   
        
        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;
       
        auto ranges_vec = preprocess_lidar(scan_msg);
	// 벡터 전처리(nan, inf, max) & 이전 벡터값과의 평균
        int closest_index = find_closest_point(ranges_vec);
        // 가장 작은 벡터 요소의 인덱스(가장 가까운 거리)
	float closest_range = ranges_vec[closest_index];
	// 원본 변경 전 closest_range 저장
        process_bubble(&ranges_vec, closest_index);
        // 벡터의 가장 작은 요소 + 버블 반지름을 기준으로 벡터의 요소를 0으로 초기화(원본 변경)
        max_gap_start, max_gep_end = find_max_gap(ranges_vec);
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
