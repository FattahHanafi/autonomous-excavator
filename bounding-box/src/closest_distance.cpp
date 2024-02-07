#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class ClosestDistance : public rclcpp::Node {
 public:
  ClosestDistance() : Node("closest_distance_node") {
    this->declare_parameter("source", "");
    this->declare_parameter("target_1", "");
    // this->declare_parameter("target_2", "");
    // this->declare_parameter("target_3", "");
    // this->declare_parameter("target_4", "");
    auto source = this->get_parameter("source").as_string();
    auto target_1 = this->get_parameter("target_1").as_string();
    // auto target_2 = this->get_parameter("target_2").as_string();
    // auto target_3 = this->get_parameter("target_3").as_string();
    // auto target_4 = this->get_parameter("target_4").as_string();

    m_distancePublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("boundingBox/Distane" + source + "_to_" + target_1, 10);
    m_sourceSubscriber = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "boundingBox/" + source, 10, std::bind(&ClosestDistance::sourceCallback, this, std::placeholders::_1));

    if (target_1.length() > 1)
      m_targetSubscriber_1 = this->create_subscription<visualization_msgs::msg::MarkerArray>(
          "boundingBox/" + target_1, 10, std::bind(&ClosestDistance::target1Callback, this, std::placeholders::_1));

    // if (target_2.length() > 1)
    //   m_targetSubscriber_1 = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //       "boundingBox/" + target_2, 10, std::bind(&ClosestDistance::target2Callback, this, std::placeholders::_1));
    //
    // if (target_3.length() > 1)
    //   m_targetSubscriber_1 = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //       "boundingBox/" + target_3, 10, std::bind(&ClosestDistance::target3Callback, this, std::placeholders::_1));
    //
    // if (target_4.length() > 1)
    //   m_targetSubscriber_1 = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //       "boundingBox/" + target_4, 10, std::bind(&ClosestDistance::target4Callback, this, std::placeholders::_1));
    //
    m_distanceArray.markers.resize(3);
    m_distanceArray.markers.at(0).ns = "dis" + source + target_1;
    m_distanceArray.markers.at(0).id = 0;
    m_distanceArray.markers.at(0).action = visualization_msgs::msg::Marker::DELETEALL;

    m_distanceArray.markers.at(2).ns = "dis" + source + target_1;
    m_distanceArray.markers.at(2).id = 2;
    m_distanceArray.markers.at(2).action = visualization_msgs::msg::Marker::ADD;
    m_distanceArray.markers.at(2).type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m_distanceArray.markers.at(2).text = "";
    m_distanceArray.markers.at(2).header.frame_id = "world";
    m_distanceArray.markers.at(2).color.r = 0.0f;
    m_distanceArray.markers.at(2).color.g = 0.0f;
    m_distanceArray.markers.at(2).color.b = 1.0f;
    m_distanceArray.markers.at(2).color.a = 1.0f;
    m_distanceArray.markers.at(2).scale.x = 0.1f;
    m_distanceArray.markers.at(2).scale.y = 0.1f;
    m_distanceArray.markers.at(2).scale.z = 0.1f;

    m_distanceArray.markers.at(1).ns = source;
    m_distanceArray.markers.at(1).id = 1;
    m_distanceArray.markers.at(1).type = visualization_msgs::msg::Marker::LINE_LIST;
    m_distanceArray.markers.at(1).action = visualization_msgs::msg::Marker::ADD;
    m_distanceArray.markers.at(1).header.frame_id = "world";
    m_distanceArray.markers.at(1).scale.x = 0.01f;
    m_distanceArray.markers.at(1).scale.y = 0.01f;
    m_distanceArray.markers.at(1).scale.z = 0.01f;
    m_distanceArray.markers.at(1).color.r = 0.0f;
    m_distanceArray.markers.at(1).color.g = 0.0f;
    m_distanceArray.markers.at(1).color.b = 1.0f;
    m_distanceArray.markers.at(1).color.a = 1.0f;
    m_distanceArray.markers.at(1).pose.position.x = 0.0f;
    m_distanceArray.markers.at(1).pose.position.y = 0.0f;
    m_distanceArray.markers.at(1).pose.position.z = 0.0f;
    m_distanceArray.markers.at(1).points.resize(2);
  }

  void sourceCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    m_source = msg->markers.at(1);
    auto pose = msg->markers.at(1).pose;
    auto it = m_source.points.begin();
    while (it != m_source.points.end()) {
      it->x += pose.position.x;
      it->y += pose.position.y;
      it->z += pose.position.z;
      ++it;
    }
    posePublisher();
  }
  void target1Callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    m_target_1 = msg->markers.at(1);
    auto pose = msg->markers.at(1).pose;
    auto it = m_target_1.points.begin();
    while (it != m_target_1.points.end()) {
      it->x += pose.position.x;
      it->y += pose.position.y;
      it->z += pose.position.z;
      ++it;
    }
  }
  // void target2Callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) { m_target_2 = msg->markers.at(1); }
  // void target3Callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) { m_target_3 = msg->markers.at(1); }
  // void target4Callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) { m_target_4 = msg->markers.at(1); }

  void posePublisher() {
    auto it_s = m_source.points.cbegin();
    RCLCPP_INFO(this->get_logger(), "%ld, %ld", m_source.points.size(), m_target_1.points.size());

    float max_dis = 1e6;
    while (it_s != m_source.points.cend()) {
      auto it_t = m_target_1.points.cbegin();
      while (it_t != m_target_1.points.cend()) {
        float dis = (it_s->x - it_t->x) * (it_s->x - it_t->x) + (it_s->y - it_t->y) * (it_s->y - it_t->y) + (it_s->z - it_t->z) * (it_s->z - it_t->z);
        if (dis < max_dis) {
          max_dis = dis;
          m_distanceArray.markers.at(1).points.at(0) = *it_s;
          m_distanceArray.markers.at(1).points.at(1) = *it_t;
        }
        ++it_t;
      }
      ++it_s;
    }
    m_distanceArray.markers.at(2).text = std::to_string(std::sqrt(max_dis));
    m_distanceArray.markers.at(2).pose.position.x =
        0.5 * (m_distanceArray.markers.at(1).points.at(0).x + m_distanceArray.markers.at(1).points.at(1).x);
    m_distanceArray.markers.at(2).pose.position.y =
        0.5 * (m_distanceArray.markers.at(1).points.at(0).y + m_distanceArray.markers.at(1).points.at(1).y);
    m_distanceArray.markers.at(2).pose.position.z =
        0.5 * (m_distanceArray.markers.at(1).points.at(0).z + m_distanceArray.markers.at(1).points.at(1).z);
    m_distancePublisher->publish(m_distanceArray);
  }

  visualization_msgs::msg::Marker m_source;
  visualization_msgs::msg::Marker m_target_1;
  visualization_msgs::msg::Marker m_target_2;
  visualization_msgs::msg::Marker m_target_3;
  visualization_msgs::msg::Marker m_target_4;
  visualization_msgs::msg::MarkerArray m_distanceArray;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_sourceSubscriber;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_distancePublisher;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_targetSubscriber_1;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_targetSubscriber_2;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_targetSubscriber_3;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_targetSubscriber_4;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosestDistance>());
  rclcpp::shutdown();
  return 0;
}
