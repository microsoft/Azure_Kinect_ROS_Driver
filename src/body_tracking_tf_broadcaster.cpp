// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>


using std::placeholders::_1;


class BodyTrackingTFBroadcaster : public rclcpp::Node
{
  public:
    BodyTrackingTFBroadcaster()
    : Node("tf_body_tracking_broadcaster")
    {
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/body_tracking_data", 10, std::bind(&BodyTrackingTFBroadcaster::callback, this, _1));


        // Initialize map with id and corresponding body part
        int index_start = 100;
        std::vector<std::string> body_tracking_vector {"Pelvis", "Spine_Naval", "Spine_Chest", "Neck", "Clavicle_left",
                                                        "Shoulder_left", "Elbow_left", "Wrist_left", "Hand_left", "Handtip_left",
                                                        "Thumb_left", "Clavicle_right", "Shoulder_right", "Elbow_right", "Wrist_right",
                                                        "Hand_right", "Handtip_right", "Thumb_right", "Hip_left", "Knee_left", "Ankle_left",
                                                        "Foot_left", "Hip_right", "Knee_right", "Ankle_right", "Foot_right", "Head", "Nose"};
        for (auto it = begin (body_tracking_vector); it != end (body_tracking_vector); ++it)
        {
            body_tracking_map_.insert(std::pair<int, std::string>(index_start, *it));
            ++index_start;
        }
    }


  private:
    geometry_msgs::msg::TransformStamped TransformPublisher(geometry_msgs::msg::Pose pose, std::string parent_frame, std::string child_frame)
    {
        tf2_ros::TransformBroadcaster tf_broadcaster = tf2_ros::TransformBroadcaster(this);
        geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();
        t.header.frame_id = parent_frame;
        t.child_frame_id = child_frame;

        t.transform.translation.x = pose.position.x;
        t.transform.translation.y = pose.position.y;
        t.transform.translation.z = pose.position.z;
        t.transform.set__rotation(pose.orientation);

        tf_broadcaster.sendTransform(t);

        return t;
    }
    void callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        for (int i = 0; i < 32; i++)
        {
            if (msg != NULL && msg->markers.size() > i)
            {
                int id = msg->markers[i].id;
                if (body_tracking_map_.find(id) != body_tracking_map_.end())
                {
                    TransformPublisher(msg->markers[i].pose, parent_frame_, body_tracking_map_.at(id));
                }
            }
        }


    }
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    std::map<int, std::string> body_tracking_map_;
    const std::string parent_frame_ = "depth_camera_link";
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyTrackingTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}