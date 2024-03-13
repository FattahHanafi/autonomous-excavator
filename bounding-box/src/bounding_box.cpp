#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class ClosestDistance : public rclcpp::Node {
public:
  ClosestDistance() : Node("bounding_box_node") {
    this->declare_parameter("title", "box");
    this->declare_parameter("frame_id", "world");
    this->declare_parameter("size_x", 1.0f);
    this->declare_parameter("size_y", 1.0f);
    this->declare_parameter("size_z", 1.0f);
    this->declare_parameter("position_x", 0.0f);
    this->declare_parameter("position_y", 0.0f);
    this->declare_parameter("position_z", 0.0f);
    this->declare_parameter("orientation_x", 0.0f);
    this->declare_parameter("orientation_y", 0.0f);
    this->declare_parameter("orientation_z", 0.0f);
    this->declare_parameter("orientation_w", 1.0f);
    this->declare_parameter("color_r", 1.0f);
    this->declare_parameter("color_g", 0.0f);
    this->declare_parameter("color_b", 0.0f);
    this->declare_parameter("color_a", 0.3f);
    this->declare_parameter("motion", 0);

    m_timer = this->create_wall_timer(
        100ms, std::bind(&ClosestDistance::posePublisher, this));
    m_boxPublisher =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "boundingBox/" + this->get_parameter("title").as_string(), 10);
    m_startTime = this->get_clock()->now();
    m_size.x = this->get_parameter("size_x").as_double();
    m_size.y = this->get_parameter("size_y").as_double();
    m_size.z = this->get_parameter("size_z").as_double();
    m_pose.position.x = this->get_parameter("position_x").as_double();
    m_pose.position.y = this->get_parameter("position_y").as_double();
    m_pose.position.z = this->get_parameter("position_z").as_double();
    m_pose.orientation.x = this->get_parameter("orientation_x").as_double();
    m_pose.orientation.y = this->get_parameter("orientation_y").as_double();
    m_pose.orientation.z = this->get_parameter("orientation_z").as_double();
    m_pose.orientation.w = this->get_parameter("orientation_w").as_double();

    m_boundaryBoxArray.markers.resize(2);

    m_boundaryBoxArray.markers.at(1).header.frame_id =
        this->get_parameter("frame_id").as_string();
    m_boundaryBoxArray.markers.at(1).ns =
        this->get_parameter("title").as_string();
    m_boundaryBoxArray.markers.at(1).id = 1;
    m_boundaryBoxArray.markers.at(1).action =
        visualization_msgs::msg::Marker::MODIFY;
    m_boundaryBoxArray.markers.at(1).type =
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m_boundaryBoxArray.markers.at(1).text =
        this->get_parameter("title").as_string();

    m_boundaryBoxArray.markers.at(1).color.r = 1.0f;
    m_boundaryBoxArray.markers.at(1).color.g = 1.0f;
    m_boundaryBoxArray.markers.at(1).color.b = 1.0f;
    m_boundaryBoxArray.markers.at(1).color.a = 0.5f;
    m_boundaryBoxArray.markers.at(1).scale.x = 0.1f;
    m_boundaryBoxArray.markers.at(1).scale.y = 0.1f;
    m_boundaryBoxArray.markers.at(1).scale.z = 0.1f;

    m_boundaryBoxArray.markers.at(0).set__header(
        m_boundaryBoxArray.markers.at(1).header);
    m_boundaryBoxArray.markers.at(0).set__pose(
        m_boundaryBoxArray.markers.at(1).pose);
    m_boundaryBoxArray.markers.at(0).color.r =
        this->get_parameter("color_r").as_double();
    m_boundaryBoxArray.markers.at(0).color.g =
        this->get_parameter("color_g").as_double();
    m_boundaryBoxArray.markers.at(0).color.b =
        this->get_parameter("color_b").as_double();
    m_boundaryBoxArray.markers.at(0).color.a =
        this->get_parameter("color_a").as_double();
    m_boundaryBoxArray.markers.at(0).ns = m_boundaryBoxArray.markers.at(1).ns;
    m_boundaryBoxArray.markers.at(0).id = 0;
    m_boundaryBoxArray.markers.at(0).type =
        visualization_msgs::msg::Marker::CUBE;
    m_boundaryBoxArray.markers.at(0).action =
        visualization_msgs::msg::Marker::ADD;
    m_boundaryBoxArray.markers.at(0).set__scale(m_size);

    m_boundaryBoxArray.markers.at(0).points.resize(8);
    m_boundaryBoxArray.markers.at(0).points.at(0).x = -0.5 * m_size.x;
    m_boundaryBoxArray.markers.at(0).points.at(0).y = -0.5 * m_size.y;
    m_boundaryBoxArray.markers.at(0).points.at(0).z = -0.5 * m_size.z;
    m_boundaryBoxArray.markers.at(0).points.at(0).x = +0.5 * m_size.x;
    m_boundaryBoxArray.markers.at(0).points.at(0).y = -0.5 * m_size.y;
    m_boundaryBoxArray.markers.at(0).points.at(0).z = -0.5 * m_size.z;
    m_boundaryBoxArray.markers.at(0).points.at(1).x = +0.5 * m_size.x;
    m_boundaryBoxArray.markers.at(0).points.at(1).y = +0.5 * m_size.y;
    m_boundaryBoxArray.markers.at(0).points.at(1).z = -0.5 * m_size.z;
    m_boundaryBoxArray.markers.at(0).points.at(3).x = -0.5 * m_size.x;
    m_boundaryBoxArray.markers.at(0).points.at(3).y = +0.5 * m_size.y;
    m_boundaryBoxArray.markers.at(0).points.at(3).z = -0.5 * m_size.z;
    m_boundaryBoxArray.markers.at(0).points.at(4).x = -0.5 * m_size.x;
    m_boundaryBoxArray.markers.at(0).points.at(4).y = -0.5 * m_size.y;
    m_boundaryBoxArray.markers.at(0).points.at(4).z = +0.5 * m_size.z;
    m_boundaryBoxArray.markers.at(0).points.at(5).x = +0.5 * m_size.x;
    m_boundaryBoxArray.markers.at(0).points.at(5).y = -0.5 * m_size.y;
    m_boundaryBoxArray.markers.at(0).points.at(5).z = +0.5 * m_size.z;
    m_boundaryBoxArray.markers.at(0).points.at(6).x = +0.5 * m_size.x;
    m_boundaryBoxArray.markers.at(0).points.at(6).y = +0.5 * m_size.y;
    m_boundaryBoxArray.markers.at(0).points.at(6).z = +0.5 * m_size.z;
    m_boundaryBoxArray.markers.at(0).points.at(7).x = -0.5 * m_size.x;
    m_boundaryBoxArray.markers.at(0).points.at(7).y = +0.5 * m_size.y;
    m_boundaryBoxArray.markers.at(0).points.at(7).z = +0.5 * m_size.z;
  }

  void posePublisher() {
    auto time = this->get_clock()->now();

    // m_distanceArray.markers.at(0).header.stamp = time;
    // m_distanceArray.markers.at(1).header.stamp = time;

    geometry_msgs::msg::Pose pose;
    float theta = time.seconds() - m_startTime.seconds();
    if (this->get_parameter("motion").as_int() == 1) {
      pose.position.x = 3.0 + 0.5 * cos(theta);
      pose.position.y = -3.0 + 2.5 * sin(theta);
      pose.position.z = 0.5 * m_size.z;
    } else if (this->get_parameter("motion").as_int() == 2) {
      pose.position.x = 6.0;
      pose.position.y = 2.5 + 2.0 * sin(theta);
      pose.position.z = 0.5;
    } else if (this->get_parameter("motion").as_int() == 3) {
      pose.position.x = 0.0;
      pose.position.y = -5.0;
      pose.position.z = -3.0;
    }

    pose.position.x += m_pose.position.x;
    pose.position.y += m_pose.position.y;
    pose.position.z += m_pose.position.z;

    m_boundaryBoxArray.markers.at(0).set__pose(pose);
    m_boundaryBoxArray.markers.at(1).set__pose(pose);
    // m_boundaryBoxArray.markers.at(1).pose.position.z += 0.5 * m_size.z;

    m_boxPublisher->publish(m_boundaryBoxArray);
  };

  visualization_msgs::msg::MarkerArray m_boundaryBoxArray;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_boxPublisher;
  rclcpp::TimerBase::SharedPtr m_timer;
  geometry_msgs::msg::Vector3 m_size;
  geometry_msgs::msg::Pose m_pose;
  rclcpp::Time m_startTime;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosestDistance>());
  rclcpp::shutdown();
  return 0;
}
