#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

//namespaces
using namespace sensor_msgs::msg;
using namespace ackermann_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;
using namespace visualization_msgs::msg;
using namespace std;
using std::placeholders::_1;

//global variables
LaserScan::SharedPtr scan_data;
int max_gap_index = 0;
struct {
    double x;
    double y;
    double yaw;
} odom;

class ftg_node : public rclcpp::Node{
    public:
        ftg_node(): Node("ftg_node"){
            drive_publisher_ = this->create_publisher<AckermannDriveStamped>("/drive", 10);
            lidar_subscriber_ = this->create_subscription<LaserScan>("/scan", 10, std::bind(&ftg_node::lidar_callback, this, _1));
            odom_subscriber_ = this->create_subscription<Odometry>("/ego_racecar/odom", 10, std::bind(&ftg_node::odom_callback, this, _1));
            vis_publisher_ = this->create_publisher<Marker>("/visualization_marker", 10);
            ackermann_publisher_ = this->create_publisher<AckermannDriveStamped>("/drive", 10);
        }

        double yaw_from_quaternion(Quaternion quat) const {
            tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getEulerYPR(yaw, pitch, roll);
            return yaw;
        }

        double rads_to_follow(int last_index, int max_gap_index) const {
            double angle_increment = 0.004351851996034384;
            int relative_point = max_gap_index - int(floor((last_index+1)/2));
            float rads = angle_increment*relative_point;

            return rads;
        }

        void lidar_callback(const LaserScan::SharedPtr msg) const {
            scan_data = msg;
            find_gap(181);
        }

        void odom_callback(const Odometry::SharedPtr msg) const {
            odom.x = msg->pose.pose.position.x;
            odom.y = msg->pose.pose.position.y;
            odom.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
        }

        vector<float> trim_filter_ranges() const {
            auto ranges = scan_data->ranges;

            vector<float>::iterator it1 = ranges.begin();
            vector<float>::iterator it2 = ranges.begin()+149;
            ranges.erase(it1, it2);

            vector<float>::iterator it3 = ranges.end()-149;
            vector<float>::iterator it4 = ranges.end();
            ranges.erase(it3, it4);

            auto last_index = int(ranges.size()-1);
            float distance = 0;
            float max_distance = 12.5;

            for(int i = 0; i<=last_index; i++){
                distance = ranges.at(i);

                if(distance > max_distance)
                    ranges.at(i) = 0;
                else
                    continue;
            }

            return ranges;
        }

        void display_gap(int last_index, int max_gap_index) const {
            float rads = rads_to_follow(last_index, max_gap_index);

            float point_location = tan(rads) * 1;

            Marker marker;
            marker.header.frame_id = "ego_racecar/base_link";
            marker.header.stamp = now();
            marker.ns = "gaps";
            marker.id = 0;
            marker.type = Marker::POINTS;
            marker.action = Marker::ADD;
            marker.frame_locked = true;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.color.g = 1.0f;
            marker.color.a = 1.0;

            Point p;
            p.x = 1.0;
            p.y = point_location;
            p.z = 0.05;

            marker.pose.position = p;
            marker.points.push_back(p);

            vis_publisher_->publish(marker);
        }

        void find_gap(int gap_size) const {
            auto ranges = trim_filter_ranges();

            auto last_index = int(ranges.size()-1);

            float max_gap = 0;

            for(int i = 0; i<=last_index-(gap_size-1); i++){
                float temp_sum = 0;
                for(int j = i; j<=i+(gap_size-1); j++){
                    temp_sum = temp_sum + ranges.at(j);
                }

                if(temp_sum > max_gap){
                    max_gap = temp_sum;
                    max_gap_index = i+int(floor(gap_size/2));
                }
                else
                    continue;
            }

            display_gap(last_index, max_gap_index);
            follow_gap(last_index, max_gap_index);
        }

        void follow_gap(int last_index, int max_gap_index) const {
            float rads = rads_to_follow(last_index, max_gap_index);

            if(abs(rads) > 0.698131701){
                rads = rads / abs(rads) * 0.698131701;
            }

            AckermannDriveStamped drive_cmd;
            drive_cmd.drive.speed = 1.5;
            drive_cmd.drive.acceleration = 0.5;
            drive_cmd.drive.steering_angle = rads;
            drive_cmd.drive.steering_angle_velocity = 0.05;
            ackermann_publisher_->publish(drive_cmd);
        }

        //field declarations
        rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_publisher_;
        rclcpp::Subscription<LaserScan>::SharedPtr lidar_subscriber_;
        rclcpp::Subscription<Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Publisher<Marker>::SharedPtr vis_publisher_;
        rclcpp::Publisher<AckermannDriveStamped>::SharedPtr ackermann_publisher_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ftg_node>());
    rclcpp::shutdown();
    return 0;
}