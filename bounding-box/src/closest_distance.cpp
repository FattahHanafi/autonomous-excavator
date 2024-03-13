#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class ClosestDistance : public rclcpp::Node {
public:
  ClosestDistance() : Node("closest_distance_node") {
    this->declare_parameter("source", "");
    this->declare_parameter("target_1", "");
    this->declare_parameter("target_2", "");
    this->declare_parameter("target_3", "");
    this->declare_parameter("target_4", "");
    auto source = this->get_parameter("source").as_string();
    auto target_1 = this->get_parameter("target_1").as_string();
    auto target_2 = this->get_parameter("target_2").as_string();
    auto target_3 = this->get_parameter("target_3").as_string();
    auto target_4 = this->get_parameter("target_4").as_string();

    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    m_distancePublisher =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "distance/" + source, 10);

    m_subscriber.push_back(
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "boundingBox/" + source, 10,
            std::bind(&ClosestDistance::Callback, this,
                      std::placeholders::_1)));
    if (target_1.size() > 1)
      m_subscriber.push_back(
          this->create_subscription<visualization_msgs::msg::MarkerArray>(
              "boundingBox/" + target_1, 10,
              std::bind(&ClosestDistance::Callback, this,
                        std::placeholders::_1)));
    // if (target_2.size() > 1)
    //   m_subscriber.push_back(
    //       this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //           "boundingBox/" + target_2, 10,
    //           std::bind(&ClosestDistance::Callback, this,
    //                     std::placeholders::_1)));
    // if (target_3.size() > 1)
    //   m_subscriber.push_back(
    //       this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //           "boundingBox/" + target_3, 10,
    //           std::bind(&ClosestDistance::Callback, this,
    //                     std::placeholders::_1)));
    // if (target_4.size() > 1)
    //   m_subscriber.push_back(
    //       this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //           "boundingBox/" + target_4, 10,
    //           std::bind(&ClosestDistance::Callback, this,
    //                     std::placeholders::_1)));

    m_distanceArray.markers.resize(2);

    m_distanceArray.markers.at(1).ns = "dis" + source;
    m_distanceArray.markers.at(1).id = 1;
    m_distanceArray.markers.at(1).action = visualization_msgs::msg::Marker::ADD;
    m_distanceArray.markers.at(1).type =
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m_distanceArray.markers.at(1).text = "";
    m_distanceArray.markers.at(1).header.frame_id = "world";
    m_distanceArray.markers.at(1).color.r = 0.0f;
    m_distanceArray.markers.at(1).color.g = 0.0f;
    m_distanceArray.markers.at(1).color.b = 1.0f;
    m_distanceArray.markers.at(1).color.a = 1.0f;
    m_distanceArray.markers.at(1).scale.x = 0.1f;
    m_distanceArray.markers.at(1).scale.y = 0.1f;
    m_distanceArray.markers.at(1).scale.z = 0.1f;

    m_distanceArray.markers.at(0).ns = "dis" + source;
    m_distanceArray.markers.at(0).id = 0;
    m_distanceArray.markers.at(0).action = visualization_msgs::msg::Marker::ADD;
    m_distanceArray.markers.at(0).type =
        visualization_msgs::msg::Marker::LINE_LIST;
    m_distanceArray.markers.at(0).header.frame_id = "world";
    m_distanceArray.markers.at(0).scale.x = 0.01f;
    m_distanceArray.markers.at(0).scale.y = 0.01f;
    m_distanceArray.markers.at(0).scale.z = 0.01f;
    m_distanceArray.markers.at(0).color.r = 0.0f;
    m_distanceArray.markers.at(0).color.g = 0.0f;
    m_distanceArray.markers.at(0).color.b = 1.0f;
    m_distanceArray.markers.at(0).color.a = 1.0f;
    m_distanceArray.markers.at(0).pose.position.x = 0.0f;
    m_distanceArray.markers.at(0).pose.position.y = 0.0f;
    m_distanceArray.markers.at(0).pose.position.z = 0.0f;
    m_distanceArray.markers.at(0).points.resize(2);
  }

  void Callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    if (count >= 0) {
      RCLCPP_INFO(this->get_logger(), "Loading Data : %d %%", 100 - count);
      --count;
      return;
    }

    auto t1 = m_tfBuffer->lookupTransform(
        "world", msg->markers.at(0).header.frame_id, tf2::TimePointZero);
    auto t2 = m_tfBuffer->lookupTransform(
        "world", msg->markers.at(0).header.frame_id, tf2::TimePointZero);
    // auto t2 = m_tfBuffer->lookupTransform("world", "map",
    // tf2::TimePointZero);
    t2.transform.translation.x = msg->markers.at(0).pose.position.x;
    t2.transform.translation.y = msg->markers.at(0).pose.position.y;
    t2.transform.translation.z = msg->markers.at(0).pose.position.z;
    t2.transform.rotation.x = msg->markers.at(0).pose.orientation.x;
    t2.transform.rotation.y = msg->markers.at(0).pose.orientation.y;
    t2.transform.rotation.z = msg->markers.at(0).pose.orientation.z;
    t2.transform.rotation.w = msg->markers.at(0).pose.orientation.w;
    //
    // RCLCPP_INFO(this->get_logger(), "t1: %f, %f, %f, %s",
    // t1.transform.translation.x, t1.transform.translation.y,
    // t1.transform.translation.z,
    //             msg->markers.at(0).header.frame_id.c_str());
    // RCLCPP_INFO(this->get_logger(), "t2: %f, %f, %f, %s",
    // t2.transform.translation.x, t2.transform.translation.y,
    // t2.transform.translation.z,
    //             msg->markers.at(0).header.frame_id.c_str());

    if (msg->markers.at(0).ns == this->get_parameter("source").as_string()) {
      m_source = msg->markers.at(0);
      for (size_t i = 0; i < m_source.points.size(); ++i) {
        tf2::doTransform(msg->markers.at(0).points.at(i), m_source.points.at(i),
                         t2);
        tf2::doTransform(m_source.points.at(i), m_source.points.at(i), t1);
      }
      m_source.header.frame_id = "world";
      posePublisher();
    }

    else {
      m_target_1 = msg->markers.at(0);
      for (size_t i = 0; i < m_target_1.points.size(); ++i) {
        // tf2::doTransform(msg->markers.at(0).points.at(i),
        //                  m_target_1.points.at(i), t2);
        t1 = m_tfBuffer->lookupTransform("world", "map", tf2::TimePointZero);
        tf2::doTransform(m_target_1.points.at(i), m_target_1.points.at(i), t1);
      }
      m_target_1.header.frame_id = "world";
    }

    // else if (msg->markers.at(0).ns ==
    //          this->get_parameter("target_1").as_string()) {
    //   m_target_1 = msg->markers.at(0);
    //   for (size_t i = 0; i < m_target_1.points.size(); ++i) {
    //     tf2::doTransform(msg->markers.at(0).points.at(i),
    //                      m_target_1.points.at(i), t2);
    //     tf2::doTransform(m_target_1.points.at(i), m_target_1.points.at(i),
    //     t1);
    //   }
    //   m_target_1.header.frame_id = "world";
    // }
    // else if (msg->markers.at(0).ns ==
    //          this->get_parameter("target_2").as_string()) {
    //   m_target_2 = msg->markers.at(0);
    //   for (size_t i = 0; i < m_target_2.points.size(); ++i) {
    //     tf2::doTransform(msg->markers.at(0).points.at(i),
    //                      m_target_2.points.at(i), t2);
    //     tf2::doTransform(m_target_2.points.at(i), m_target_2.points.at(i),
    //     t1);
    //   }
    //   m_target_2.header.frame_id = "world";
    // }
    // else if (msg->markers.at(0).ns ==
    //          this->get_parameter("target_3").as_string()) {
    //   m_target_3 = msg->markers.at(0);
    //   for (size_t i = 0; i < m_target_3.points.size(); ++i) {
    //     tf2::doTransform(msg->markers.at(0).points.at(i),
    //                      m_target_3.points.at(i), t2);
    //     tf2::doTransform(m_target_3.points.at(i), m_target_3.points.at(i),
    //     t1);
    //   }
    //   m_target_3.header.frame_id = "world";
    // }
    // else if (msg->markers.at(0).ns ==
    //          this->get_parameter("target_4").as_string()) {
    //   m_target_4 = msg->markers.at(0);
    //   for (size_t i = 0; i < m_target_4.points.size(); ++i) {
    //     tf2::doTransform(msg->markers.at(0).points.at(i),
    //                      m_target_3.points.at(i), t2);
    //     tf2::doTransform(m_target_3.points.at(i), m_target_3.points.at(i),
    //     t1);
    //   }
    //   m_target_4.header.frame_id = "world";
    // }
  }

  void posePublisher() {
    auto it_s = m_source.points.cbegin();

    float max_dis = 1e6;
    while (it_s != m_source.points.cend()) {
      auto it_t = m_target_1.points.cbegin();
      while (it_t != m_target_1.points.cend()) {
        float dis = (it_s->x - it_t->x) * (it_s->x - it_t->x) +
                    (it_s->y - it_t->y) * (it_s->y - it_t->y) +
                    (it_s->z - it_t->z) * (it_s->z - it_t->z);
        if (dis < max_dis) {
          max_dis = dis;
          m_distanceArray.markers.at(0).points.at(0) = *it_s;
          m_distanceArray.markers.at(0).points.at(1) = *it_t;
        }
        ++it_t;
      }

      // it_t = m_target_2.points.cbegin();
      // while (it_t != m_target_2.points.cend()) {
      //   float dis = (it_s->x - it_t->x) * (it_s->x - it_t->x) +
      //               (it_s->y - it_t->y) * (it_s->y - it_t->y) +
      //               (it_s->z - it_t->z) * (it_s->z - it_t->z);
      //   if (dis < max_dis) {
      //     max_dis = dis;
      //     m_distanceArray.markers.at(0).points.at(0) = *it_s;
      //     m_distanceArray.markers.at(0).points.at(1) = *it_t;
      //   }
      //   ++it_t;
      // }
      //
      // it_t = m_target_3.points.cbegin();
      // while (it_t != m_target_3.points.cend()) {
      //   float dis = (it_s->x - it_t->x) * (it_s->x - it_t->x) +
      //               (it_s->y - it_t->y) * (it_s->y - it_t->y) +
      //               (it_s->z - it_t->z) * (it_s->z - it_t->z);
      //   if (dis < max_dis) {
      //     max_dis = dis;
      //     m_distanceArray.markers.at(0).points.at(0) = *it_s;
      //     m_distanceArray.markers.at(0).points.at(1) = *it_t;
      //   }
      //   ++it_t;
      // }
      //
      // it_t = m_target_4.points.cbegin();
      // while (it_t != m_target_4.points.cend()) {
      //   float dis = (it_s->x - it_t->x) * (it_s->x - it_t->x) +
      //               (it_s->y - it_t->y) * (it_s->y - it_t->y) +
      //               (it_s->z - it_t->z) * (it_s->z - it_t->z);
      //   if (dis < max_dis) {
      //     max_dis = dis;
      //     m_distanceArray.markers.at(0).points.at(0) = *it_s;
      //     m_distanceArray.markers.at(0).points.at(1) = *it_t;
      //   }
      //   ++it_t;
      // }

      ++it_s;
    }
    m_distanceArray.markers.at(1).text = std::to_string(std::sqrt(max_dis));
    m_distanceArray.markers.at(1).pose.position.x =
        0.5 * (m_distanceArray.markers.at(0).points.at(0).x +
               m_distanceArray.markers.at(0).points.at(1).x);
    m_distanceArray.markers.at(1).pose.position.y =
        0.5 * (m_distanceArray.markers.at(0).points.at(0).y +
               m_distanceArray.markers.at(0).points.at(1).y);
    m_distanceArray.markers.at(1).pose.position.z =
        0.5 * (m_distanceArray.markers.at(0).points.at(0).z +
               m_distanceArray.markers.at(0).points.at(1).z);
    m_distancePublisher->publish(m_distanceArray);
  }

  std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;

  visualization_msgs::msg::Marker m_source;
  visualization_msgs::msg::Marker m_target_1;
  visualization_msgs::msg::Marker m_target_2;
  visualization_msgs::msg::Marker m_target_3;
  visualization_msgs::msg::Marker m_target_4;
  visualization_msgs::msg::MarkerArray m_distanceArray;
  std::vector<
      rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
      m_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_distancePublisher;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_targetSubscriber_1;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_targetSubscriber_2;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_targetSubscriber_3;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_targetSubscriber_4;

  int32_t count = 100;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosestDistance>());
  rclcpp::shutdown();
  return 0;
}
