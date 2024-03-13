#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>

using namespace std::chrono_literals;

class ClosestDistance : public rclcpp::Node {
public:
  ClosestDistance() : Node("zed_bounding_box_node") {
    this->declare_parameter("title", "zed");
    this->declare_parameter("color_r", 1.0f);
    this->declare_parameter("color_g", 0.0f);
    this->declare_parameter("color_b", 0.0f);
    this->declare_parameter("color_a", 0.3f);

    m_objectSubscriber =
        this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
            "/zed/zed_node/body_trk/skeletons", 10,
            std::bind(&ClosestDistance::bbPublisher, this,
                      std::placeholders::_1));

    m_boxPublisher =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "boundingBox/" + this->get_parameter("title").as_string(), 10);
    m_boundaryBoxArray.markers.resize(2);

    m_boundaryBoxArray.markers.at(1).action =
        visualization_msgs::msg::Marker::MODIFY;
    m_boundaryBoxArray.markers.at(1).type =
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

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
    m_boundaryBoxArray.markers.at(0).type =
        visualization_msgs::msg::Marker::CUBE;
    m_boundaryBoxArray.markers.at(0).action =
        visualization_msgs::msg::Marker::ADD;
    m_boundaryBoxArray.markers.at(0).points.resize(8);
    m_boundaryBoxArray.markers.at(0).lifetime.sec = 1;
    m_boundaryBoxArray.markers.at(1).lifetime.sec = 1;
  }

  void bbPublisher(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) {
    m_boundaryBoxArray.markers.at(0).set__header(msg->header);
    m_boundaryBoxArray.markers.at(1).set__header(msg->header);
    // m_boundaryBoxArray.markers.at(0).header.frame_id = msg->header.frame_id;
    // m_boundaryBoxArray.markers.at(1).header.frame_id = msg->header.frame_id;
    for (auto obj : msg->objects) {
      geometry_msgs::msg::Pose pose;
      // RCLCPP_INFO(this->get_logger(), "%hd", obj.label_id);
      m_boundaryBoxArray.markers.at(1).text = obj.label;
      m_boundaryBoxArray.markers.at(0).ns = obj.label;
      m_boundaryBoxArray.markers.at(1).ns = obj.label;
      m_boundaryBoxArray.markers.at(0).id = obj.label_id;
      m_boundaryBoxArray.markers.at(1).id = 1000 + obj.label_id;

      pose.position.x = obj.position.at(0);
      pose.position.y = obj.position.at(1);
      pose.position.z = obj.position.at(2);

      m_size.x = obj.dimensions_3d.at(0);
      m_size.y = obj.dimensions_3d.at(2);
      m_size.z = obj.dimensions_3d.at(1);

      m_boundaryBoxArray.markers.at(0).set__scale(m_size);

      auto p = m_boundaryBoxArray.markers.at(0).points.begin();
      for (auto kp : obj.bounding_box_3d.corners) {
        p->x = kp.kp.at(0);
        p->y = kp.kp.at(2);
        p->z = kp.kp.at(1);
        ++p;
      }

      // m_boundaryBoxArray.markers.at(0).points.at(0).x =
      //     -0.5 * m_size.x + obj.position.at(0);
      // m_boundaryBoxArray.markers.at(0).points.at(0).y =
      //     -0.5 * m_size.y + obj.position.at(1);
      // m_boundaryBoxArray.markers.at(0).points.at(0).z =
      //     -0.5 * m_size.z + obj.position.at(2);
      // m_boundaryBoxArray.markers.at(0).points.at(1).x =
      //     +0.5 * m_size.x + obj.position.at(0);
      // m_boundaryBoxArray.markers.at(0).points.at(1).y =
      //     -0.5 * m_size.y + obj.position.at(2);
      // m_boundaryBoxArray.markers.at(0).points.at(1).z =
      //     -0.5 * m_size.z + obj.position.at(1);
      // m_boundaryBoxArray.markers.at(0).points.at(2).x =
      //     +0.5 * m_size.x + obj.position.at(0);
      // m_boundaryBoxArray.markers.at(0).points.at(2).y =
      //     +0.5 * m_size.y + obj.position.at(2);
      // m_boundaryBoxArray.markers.at(0).points.at(2).z =
      //     -0.5 * m_size.z + obj.position.at(1);
      // m_boundaryBoxArray.markers.at(0).points.at(3).x =
      //     -0.5 * m_size.x + obj.position.at(0);
      // m_boundaryBoxArray.markers.at(0).points.at(3).y =
      //     +0.5 * m_size.y + obj.position.at(2);
      // m_boundaryBoxArray.markers.at(0).points.at(3).z =
      //     -0.5 * m_size.z + obj.position.at(1);
      // m_boundaryBoxArray.markers.at(0).points.at(4).x =
      //     -0.5 * m_size.x + obj.position.at(0);
      // m_boundaryBoxArray.markers.at(0).points.at(4).y =
      //     -0.5 * m_size.y + obj.position.at(2);
      // m_boundaryBoxArray.markers.at(0).points.at(4).z =
      //     +0.5 * m_size.z + obj.position.at(1);
      // m_boundaryBoxArray.markers.at(0).points.at(5).x =
      //     +0.5 * m_size.x + obj.position.at(0);
      // m_boundaryBoxArray.markers.at(0).points.at(5).y =
      //     -0.5 * m_size.y + obj.position.at(2);
      // m_boundaryBoxArray.markers.at(0).points.at(5).z =
      //     +0.5 * m_size.z + obj.position.at(1);
      // m_boundaryBoxArray.markers.at(0).points.at(6).x =
      //     +0.5 * m_size.x + obj.position.at(0);
      // m_boundaryBoxArray.markers.at(0).points.at(6).y =
      //     +0.5 * m_size.y + obj.position.at(2);
      // m_boundaryBoxArray.markers.at(0).points.at(6).z =
      //     +0.5 * m_size.z + obj.position.at(1);
      // m_boundaryBoxArray.markers.at(0).points.at(7).x =
      //     -0.5 * m_size.x + obj.position.at(0);
      // m_boundaryBoxArray.markers.at(0).points.at(7).y =
      //     +0.5 * m_size.y + obj.position.at(2);
      // m_boundaryBoxArray.markers.at(0).points.at(7).z =
      //     +0.5 * m_size.z + obj.position.at(1);

      m_boundaryBoxArray.markers.at(0).set__pose(pose);
      m_boundaryBoxArray.markers.at(1).set__pose(pose);
      m_boxPublisher->publish(m_boundaryBoxArray);
    }
  };

  visualization_msgs::msg::MarkerArray m_boundaryBoxArray;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_boxPublisher;
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr
      m_objectSubscriber;
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
