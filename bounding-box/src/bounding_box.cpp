#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class ClosestDistance : public rclcpp::Node {
 public:
  ClosestDistance() : Node("bounding_box_node") {
    this->declare_parameter("title", "world");
    this->declare_parameter("motion", 0);
    this->declare_parameter("size_x", 0.5f);
    this->declare_parameter("size_y", 0.5f);
    this->declare_parameter("size_z", 2.0f);

    m_Timer = this->create_wall_timer(100ms, std::bind(&ClosestDistance::posePublisher, this));
    m_boxPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("boundingBox/" + this->get_parameter("title").as_string(), 10);
    m_startTime = this->get_clock()->now();
    m_distanceArray.markers.resize(3);
    m_distanceArray.markers.at(0).ns = this->get_parameter("title").as_string();
    m_distanceArray.markers.at(0).id = 0;
    m_distanceArray.markers.at(0).action = visualization_msgs::msg::Marker::DELETEALL;

    m_distanceArray.markers.at(2).ns = this->get_parameter("title").as_string();
    m_distanceArray.markers.at(2).id = 2;
    m_distanceArray.markers.at(2).action = visualization_msgs::msg::Marker::ADD;
    m_distanceArray.markers.at(2).type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m_distanceArray.markers.at(2).text = this->get_parameter("title").as_string();
    m_size.x = this->get_parameter("size_x").as_double();
    m_size.y = this->get_parameter("size_y").as_double();
    m_size.z = this->get_parameter("size_z").as_double();
    m_distanceArray.markers.at(2).header.frame_id = "world";
    m_distanceArray.markers.at(2).color.r = 1.0f;
    m_distanceArray.markers.at(2).color.g = 0.0f;
    m_distanceArray.markers.at(2).color.b = 0.0f;
    m_distanceArray.markers.at(2).color.a = 1.0f;
    m_distanceArray.markers.at(2).scale.x = 0.1f;
    m_distanceArray.markers.at(2).scale.y = 0.1f;
    m_distanceArray.markers.at(2).scale.z = 0.1f;

    m_boxMessage.ns = this->get_parameter("title").as_string();
    m_boxMessage.id = 1;
    m_boxMessage.type = visualization_msgs::msg::Marker::LINE_LIST;
    m_boxMessage.action = visualization_msgs::msg::Marker::ADD;
    m_boxMessage.header.frame_id = "world";
    m_boxMessage.scale.x = 0.01f;
    m_boxMessage.scale.y = 0.01f;
    m_boxMessage.scale.z = 0.01f;
    m_boxMessage.color.r = 1.0f;
    m_boxMessage.color.g = 0.0f;
    m_boxMessage.color.b = 0.0f;
    m_boxMessage.color.a = 1.0f;
    m_boxMessage.pose.position.x = 0.0f;
    m_boxMessage.pose.position.y = 0.0f;
    m_boxMessage.pose.position.z = +0.5 * m_size.z;
    m_boxMessage.points.resize(12 * 2);
    std::array<geometry_msgs::msg::Point, 8> p;
    p.at(0).x = -0.5 * m_size.x;
    p.at(0).y = -0.5 * m_size.y;
    p.at(0).z = -0.5 * m_size.z;
    p.at(1).x = +0.5 * m_size.x;
    p.at(1).y = -0.5 * m_size.y;
    p.at(1).z = -0.5 * m_size.z;
    p.at(2).x = +0.5 * m_size.x;
    p.at(2).y = +0.5 * m_size.y;
    p.at(2).z = -0.5 * m_size.z;
    p.at(3).x = -0.5 * m_size.x;
    p.at(3).y = +0.5 * m_size.y;
    p.at(3).z = -0.5 * m_size.z;
    p.at(4).x = -0.5 * m_size.x;
    p.at(4).y = -0.5 * m_size.y;
    p.at(4).z = +0.5 * m_size.z;
    p.at(5).x = +0.5 * m_size.x;
    p.at(5).y = -0.5 * m_size.y;
    p.at(5).z = +0.5 * m_size.z;
    p.at(6).x = +0.5 * m_size.x;
    p.at(6).y = +0.5 * m_size.y;
    p.at(6).z = +0.5 * m_size.z;
    p.at(7).x = -0.5 * m_size.x;
    p.at(7).y = +0.5 * m_size.y;
    p.at(7).z = +0.5 * m_size.z;

    m_boxMessage.points.at(0) = p.at(0);
    m_boxMessage.points.at(1) = p.at(1);
    m_boxMessage.points.at(2) = p.at(1);
    m_boxMessage.points.at(3) = p.at(2);
    m_boxMessage.points.at(4) = p.at(2);
    m_boxMessage.points.at(5) = p.at(3);
    m_boxMessage.points.at(6) = p.at(3);
    m_boxMessage.points.at(7) = p.at(0);
    m_boxMessage.points.at(8) = p.at(0);
    m_boxMessage.points.at(9) = p.at(4);
    m_boxMessage.points.at(10) = p.at(1);
    m_boxMessage.points.at(11) = p.at(5);
    m_boxMessage.points.at(12) = p.at(2);
    m_boxMessage.points.at(13) = p.at(6);
    m_boxMessage.points.at(14) = p.at(3);
    m_boxMessage.points.at(15) = p.at(7);
    m_boxMessage.points.at(16) = p.at(4);
    m_boxMessage.points.at(17) = p.at(5);
    m_boxMessage.points.at(18) = p.at(5);
    m_boxMessage.points.at(19) = p.at(6);
    m_boxMessage.points.at(20) = p.at(6);
    m_boxMessage.points.at(21) = p.at(7);
    m_boxMessage.points.at(22) = p.at(7);
    m_boxMessage.points.at(23) = p.at(4);

    m_distanceArray.markers.at(1) = m_boxMessage;
  }

  void posePublisher() {
    auto time = this->get_clock()->now();
    geometry_msgs::msg::Pose pose;
    float theta = time.seconds() - m_startTime.seconds();
    if (this->get_parameter("motion").as_int() == 1) {
      pose.position.x = 3.0 + 0.5 * cos(theta);
      pose.position.y = -5.0 + 0.5 * sin(theta);
      pose.position.z = 0.5 * m_size.z;
      // pose.orientation.z = sin(0.5 * theta);
      // pose.orientation.w = cos(0.5 * theta);
    } else if (this->get_parameter("motion").as_int() == 2) {
      pose.position.x = 6.0;
      pose.position.y = 2.5 + 2.0 * sin(theta);
      pose.position.z = 0.5;
    } else if (this->get_parameter("motion").as_int() == 3) {
      pose.position.x = 0.0;
      pose.position.y = -5.0;
      pose.position.z = -3.0;
    }

    m_distanceArray.markers.at(1).pose = pose;
    m_distanceArray.markers.at(2).pose = pose;
    m_distanceArray.markers.at(2).pose.position.z += 0.5 * m_size.z;

    m_boxPublisher->publish(m_distanceArray);
  };

  visualization_msgs::msg::Marker m_boxMessage;
  visualization_msgs::msg::MarkerArray m_distanceArray;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_boxPublisher;
  rclcpp::TimerBase::SharedPtr m_Timer;
  geometry_msgs::msg::Vector3 m_size;
  rclcpp::Time m_startTime;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosestDistance>());
  rclcpp::shutdown();
  return 0;
}
