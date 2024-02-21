#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>
#include <thrust/host_vector.h>

#include <array>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

template <typename T>
struct Vec3 {
  T x, y, z;
  Vec3(const T x, const T y, const T z) : x(x), y(y), z(z){};
  uint32_t total() const { return x * y * z; }

  Vec3<T> operator*(const T scale) {
    Vec3<T> result(*this);
    result->x *= scale;
    result->y *= scale;
    result->z *= scale;
    return result;
  }

  Vec3<T> operator+(const Vec3<T>& v2) {
    Vec3<T> result(*this);
    result.x += v2.x;
    result.y += v2.y;
    result.z += v2.z;
    return result;
  };
};

__global__ void d_updateHeights(const float time, float* d_heights) {
  const uint32_t i = blockIdx.x;
  const uint32_t j = threadIdx.x;
  const uint32_t idx = i * blockDim.x + j;
  if (i < 50 || i > (gridDim.x - 50) || j < 50 || j > (blockDim.x - 50))
    d_heights[idx] = 0.01;
  else
    d_heights[idx] = 0.5 + 0.2 * sin(time + 2.0 * 3.14 * float(i) / gridDim.x) * cos(2.0 * 3.14 * float(j) / blockDim.x);
}

__global__ void d_updateVolumes(const uint8_t* d_groundCubes, const uint8_t* d_bucketCubes, const float* d_sampleVolumes, float* d_groundVolumes,
                                float* d_bucketVolumes) {
  const uint32_t idx = blockIdx.x * gridDim.y * blockDim.x + blockIdx.y * blockDim.x + threadIdx.x;
  d_groundVolumes[idx] = d_sampleVolumes[d_groundCubes[idx]];
  d_bucketVolumes[idx] = d_sampleVolumes[d_bucketCubes[idx]];
}

__global__ void d_updateNodes(const float* d_heights, const float size, bool* d_groundNodes, bool* d_bucketNodes, const float min_x,
                              const float max_x, const float min_y, const float max_y, const float min_z, const float max_z) {
  const uint32_t idx = blockIdx.x * gridDim.y * blockDim.x + blockIdx.y * blockDim.x + threadIdx.x;
  const uint32_t h_idx = blockIdx.x * gridDim.y + blockIdx.y;
  float z = threadIdx.x * size;
  if (!d_bucketNodes[idx]) d_groundNodes[idx] = d_heights[h_idx] >= z;

  if (d_groundNodes[idx]) {
    float p_z = threadIdx.x * size;
    float p_y = blockIdx.y * size;
    float p_x = blockIdx.x * size;
    if (p_x > min_x && p_x < max_x && p_y > min_y && p_y < max_y && p_z > min_z && p_z < max_z) {
      d_bucketNodes[idx] = 1;
      d_groundNodes[idx] = 0;
      // d_bucketNodes[idx] = !d_groundNodes[idx]};
    }
  }
};

__global__ void d_updateCubes(const bool* d_groundNodes, const bool* d_bucketNodes, uint8_t* d_groundCubes, uint8_t* d_bucketCubes) {
  const uint32_t i = blockIdx.x;
  const uint32_t j = blockIdx.y;
  const uint32_t k = threadIdx.x;

  const uint32_t idx = i * gridDim.y * blockDim.x + j * blockDim.x + k;

  const uint32_t n_idx[] = {(i + 0) * (gridDim.y + 1) * (blockDim.x + 1) + (j + 0) * (blockDim.x + 1) + (k + 0),
                            (i + 1) * (gridDim.y + 1) * (blockDim.x + 1) + (j + 0) * (blockDim.x + 1) + (k + 0),
                            (i + 1) * (gridDim.y + 1) * (blockDim.x + 1) + (j + 1) * (blockDim.x + 1) + (k + 0),
                            (i + 0) * (gridDim.y + 1) * (blockDim.x + 1) + (j + 1) * (blockDim.x + 1) + (k + 0),
                            (i + 0) * (gridDim.y + 1) * (blockDim.x + 1) + (j + 0) * (blockDim.x + 1) + (k + 1),
                            (i + 1) * (gridDim.y + 1) * (blockDim.x + 1) + (j + 0) * (blockDim.x + 1) + (k + 1),
                            (i + 1) * (gridDim.y + 1) * (blockDim.x + 1) + (j + 1) * (blockDim.x + 1) + (k + 1),
                            (i + 0) * (gridDim.y + 1) * (blockDim.x + 1) + (j + 1) * (blockDim.x + 1) + (k + 1)};

  uint8_t res = 0;
  for (int8_t i = 7; i >= 0; --i) res += (d_groundNodes[n_idx[i]] << i);
  d_groundCubes[idx] = res;
  res = 0;
  for (int8_t i = 7; i >= 0; --i) res += (d_bucketNodes[n_idx[i]] << i);
  d_bucketCubes[idx] = res;
};

using namespace std::chrono_literals;

class MarchingCubes : public rclcpp::Node {
 public:
  MarchingCubes(const uint32_t x_count, const uint32_t y_count, const uint32_t z_count, const float size) : Node("marching_cube_node") {
    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    // m_Timer = this->create_wall_timer(200ms, std::bind(&MarchingCubes::callback, this));
    m_markerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("marchingCubes", 10);
    m_bucketSubsciber = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "boundingBox/Excavator_bucket", 10, std::bind(&MarchingCubes::callback, this, std::placeholders::_1));
    m_markerArray.markers.resize(3);
    m_markerArray.markers.at(0).ns = "mc";
    m_markerArray.markers.at(0).id = 0;
    m_markerArray.markers.at(0).action = visualization_msgs::msg::Marker::DELETEALL;

    m_markerArray.markers.at(1).ns = "mc_ground";
    m_markerArray.markers.at(1).id = 1;
    m_markerArray.markers.at(1).type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m_markerArray.markers.at(1).action = visualization_msgs::msg::Marker::ADD;
    m_markerArray.markers.at(1).header.frame_id = "container";
    m_markerArray.markers.at(1).scale.x = 1.0f;
    m_markerArray.markers.at(1).scale.y = 1.0f;
    m_markerArray.markers.at(1).scale.z = 1.0f;
    m_markerArray.markers.at(1).color.r = 1.0f;
    m_markerArray.markers.at(1).color.g = 0.5f;
    m_markerArray.markers.at(1).color.b = 0.0f;
    m_markerArray.markers.at(1).color.a = 1.0f;
    // m_markerArray.markers.at(1).pose.position.x = 1.0f;
    // m_markerArray.markers.at(1).pose.position.y = -0.5 * size * y_count;
    // m_markerArray.markers.at(1).pose.position.z = -1.0f;
    m_markerArray.markers.at(1).pose.position.x = 0.0f;
    m_markerArray.markers.at(1).pose.position.y = 0.0f;
    m_markerArray.markers.at(1).pose.position.z = 0.0f;

    m_markerArray.markers.at(2).ns = "mc_bucket";
    m_markerArray.markers.at(2).id = 2;
    m_markerArray.markers.at(2).type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m_markerArray.markers.at(2).action = visualization_msgs::msg::Marker::ADD;
    m_markerArray.markers.at(2).header.frame_id = "container";
    m_markerArray.markers.at(2).scale.x = 1.0f;
    m_markerArray.markers.at(2).scale.y = 1.0f;
    m_markerArray.markers.at(2).scale.z = 1.0f;
    m_markerArray.markers.at(2).color.r = 0.0f;
    m_markerArray.markers.at(2).color.g = 1.0f;
    m_markerArray.markers.at(2).color.b = 0.0f;
    m_markerArray.markers.at(2).color.a = 1.0f;
    m_markerArray.markers.at(2).pose.position.x = 0.0f;
    m_markerArray.markers.at(2).pose.position.y = 0.0f;
    m_markerArray.markers.at(2).pose.position.z = 0.0f;

    m_size = size;
    m_cubeCount.x = x_count;
    m_cubeCount.y = y_count;
    m_cubeCount.z = z_count;
    m_nodeCount.x = x_count + 1;
    m_nodeCount.y = y_count + 1;
    m_nodeCount.z = z_count + 1;

    d_groundCubes.resize(m_cubeCount.total());
    thrust::fill(d_groundCubes.begin(), d_groundCubes.end(), 0);
    h_groundCubes.resize(m_cubeCount.total());
    thrust::fill(h_groundCubes.begin(), h_groundCubes.end(), 0);
    d_groundVolumes.resize(m_cubeCount.total());
    thrust::fill(d_groundVolumes.begin(), d_groundVolumes.end(), 0.0f);
    d_groundNodes.resize(m_nodeCount.total());
    thrust::fill(d_groundNodes.begin(), d_groundNodes.end(), 0);
    d_groundHeights.resize(m_nodeCount.x * m_nodeCount.y);

    d_bucketCubes.resize(m_cubeCount.total());
    thrust::fill(d_bucketCubes.begin(), d_bucketCubes.end(), 0);
    h_bucketCubes.resize(m_cubeCount.total());
    thrust::fill(h_bucketCubes.begin(), h_bucketCubes.end(), 0);
    d_bucketVolumes.resize(m_cubeCount.total());
    thrust::fill(d_bucketVolumes.begin(), d_bucketVolumes.end(), 0.0f);
    d_bucketNodes.resize(m_nodeCount.total());
    thrust::fill(d_bucketNodes.begin(), d_bucketNodes.end(), 0);

    m_startTime = this->get_clock()->now().seconds();

    cudaDeviceSynchronize();
    updateHeight();
    updateMessage();
  };

  void updateHeight() {
    auto time = this->get_clock()->now();
    d_updateHeights<<<m_nodeCount.x, m_nodeCount.y>>>(float(time.seconds() - m_startTime), thrust::raw_pointer_cast(d_groundHeights.data()));
    cudaDeviceSynchronize();
  }

  void callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    if (count >= 0) {
      RCLCPP_INFO(this->get_logger(), "Loading Data : %d %%", 100 - count);
      --count;
      return;
    }
    auto t1 = m_tfBuffer->lookupTransform("container", msg->markers.at(0).header.frame_id, tf2::TimePointZero);
    auto t2 = m_tfBuffer->lookupTransform("container", msg->markers.at(0).header.frame_id, tf2::TimePointZero);
    t2.transform.translation.x = msg->markers.at(0).pose.position.x;
    t2.transform.translation.y = msg->markers.at(0).pose.position.y;
    t2.transform.translation.z = msg->markers.at(0).pose.position.z;
    t2.transform.rotation.x = msg->markers.at(0).pose.orientation.x;
    t2.transform.rotation.y = msg->markers.at(0).pose.orientation.y;
    t2.transform.rotation.z = msg->markers.at(0).pose.orientation.z;
    t2.transform.rotation.w = msg->markers.at(0).pose.orientation.w;

    visualization_msgs::msg::Marker bucketPoints;
    bucketPoints.set__points(msg->markers.at(0).points);

    for (size_t i = 0; i < bucketPoints.points.size(); ++i) {
      tf2::doTransform(msg->markers.at(0).points.at(i), bucketPoints.points.at(i), t2);
      tf2::doTransform(bucketPoints.points.at(i), bucketPoints.points.at(i), t1);
    }

    float x[2] = {1e6, -1e6};
    float y[2] = {1e6, -1e6};
    float z[2] = {1e6, -1e6};
    for (uint8_t i = 0; i < 8; ++i) {
      x[0] = std::min(x[0], float(bucketPoints.points.at(i).x));
      x[1] = std::max(x[1], float(bucketPoints.points.at(i).x));
      y[0] = std::min(y[0], float(bucketPoints.points.at(i).y));
      y[1] = std::max(y[1], float(bucketPoints.points.at(i).y));
      z[0] = std::min(z[0], float(bucketPoints.points.at(i).z));
      z[1] = std::max(z[1], float(bucketPoints.points.at(i).z));
    }
    updateNodes(x[0], x[1], y[0], y[1], z[0], z[1]);
    updateCubes();
    updateVolumes();

    updateMessage();
    // auto start = std::chrono::steady_clock::now();
    // auto end = std::chrono::steady_clock::now();
    // auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // RCLCPP_INFO(this->get_logger(), "%f took %ld ms, total triangles : %ld", m_totalVolume, dur, m_markerArray.markers.at(1).points.size());
  };

  void updateVolumes() {
    d_updateVolumes<<<dim3(m_cubeCount.x, m_cubeCount.y, 1), m_cubeCount.z>>>(
        thrust::raw_pointer_cast(d_groundCubes.data()), thrust::raw_pointer_cast(d_bucketCubes.data()),
        thrust::raw_pointer_cast(d_sampleVolumes.data()), thrust::raw_pointer_cast(d_groundVolumes.data()),
        thrust::raw_pointer_cast(d_bucketVolumes.data()));
    cudaDeviceSynchronize();
    m_totalGroundVolume = thrust::reduce(d_groundVolumes.cbegin(), d_groundVolumes.cend(), 0.0f);
    m_totalBucketVolume = thrust::reduce(d_bucketVolumes.cbegin(), d_bucketVolumes.cend(), 0.0f);
  };

  void updateNodes(const double min_x, const double max_x, const double min_y, const double max_y, const double min_z, const double max_z) {
    d_updateNodes<<<dim3(m_nodeCount.x, m_nodeCount.y, 1), m_nodeCount.z>>>(
        thrust::raw_pointer_cast(d_groundHeights.data()), m_size, thrust::raw_pointer_cast(d_groundNodes.data()),
        thrust::raw_pointer_cast(d_bucketNodes.data()), min_x, max_x, min_y, max_y, min_z, max_z);
    cudaDeviceSynchronize();
  };

  void updateCubes() {
    d_updateCubes<<<dim3(m_cubeCount.x, m_cubeCount.y, 1), m_cubeCount.z>>>(
        thrust::raw_pointer_cast(d_groundNodes.data()), thrust::raw_pointer_cast(d_bucketNodes.data()),
        thrust::raw_pointer_cast(d_groundCubes.data()), thrust::raw_pointer_cast(d_bucketCubes.data()));
    cudaDeviceSynchronize();
  };

  void updateMessage() {
    auto time = this->get_clock()->now();

    geometry_msgs::msg::Point p0, p1, p2;

    thrust::copy(d_groundCubes.cbegin(), d_groundCubes.cend(), h_groundCubes.begin());
    cudaDeviceSynchronize();
    m_markerArray.markers.at(1).points.clear();
    m_markerArray.markers.at(1).header.stamp = time;
    for (uint32_t i = 0; i < m_cubeCount.x; ++i)
      for (uint32_t j = 0; j < m_cubeCount.y; ++j)
        for (uint32_t k = 0; k < m_cubeCount.z; ++k) {
          const uint32_t c_idx = i * m_cubeCount.y * m_cubeCount.z + j * m_cubeCount.z + k;
          const uint8_t c_type = h_groundCubes[c_idx];
          uint32_t t_idx = c_type * 16;
          int8_t p = m_sampleTriangles[t_idx];
          while (p >= 0) {
            p0.x = (float(i) + m_points.at(p * 3 + 0)) * m_size;
            p0.y = (float(j) + m_points.at(p * 3 + 1)) * m_size;
            p0.z = (float(k) + m_points.at(p * 3 + 2)) * m_size;
            ++t_idx;
            p = m_sampleTriangles[t_idx];
            p1.x = (float(i) + m_points.at(p * 3 + 0)) * m_size;
            p1.y = (float(j) + m_points.at(p * 3 + 1)) * m_size;
            p1.z = (float(k) + m_points.at(p * 3 + 2)) * m_size;
            ++t_idx;
            p = m_sampleTriangles[t_idx];
            p2.x = (float(i) + m_points.at(p * 3 + 0)) * m_size;
            p2.y = (float(j) + m_points.at(p * 3 + 1)) * m_size;
            p2.z = (float(k) + m_points.at(p * 3 + 2)) * m_size;
            m_markerArray.markers.at(1).points.push_back(p2);
            m_markerArray.markers.at(1).points.push_back(p1);
            m_markerArray.markers.at(1).points.push_back(p0);

            ++t_idx;
            p = m_sampleTriangles[t_idx];
          }
        }

    thrust::copy(d_bucketCubes.cbegin(), d_bucketCubes.cend(), h_bucketCubes.begin());
    cudaDeviceSynchronize();
    m_markerArray.markers.at(2).points.clear();
    m_markerArray.markers.at(2).header.stamp = time;
    for (uint32_t i = 0; i < m_cubeCount.x; ++i)
      for (uint32_t j = 0; j < m_cubeCount.y; ++j)
        for (uint32_t k = 0; k < m_cubeCount.z; ++k) {
          const uint32_t c_idx = i * m_cubeCount.y * m_cubeCount.z + j * m_cubeCount.z + k;
          const uint8_t c_type = h_bucketCubes[c_idx];
          uint32_t t_idx = c_type * 16;
          int8_t p = m_sampleTriangles[t_idx];
          while (p >= 0) {
            p0.x = (float(i) + m_points.at(p * 3 + 0)) * m_size;
            p0.y = (float(j) + m_points.at(p * 3 + 1)) * m_size;
            p0.z = (float(k) + m_points.at(p * 3 + 2)) * m_size;
            ++t_idx;
            p = m_sampleTriangles[t_idx];
            p1.x = (float(i) + m_points.at(p * 3 + 0)) * m_size;
            p1.y = (float(j) + m_points.at(p * 3 + 1)) * m_size;
            p1.z = (float(k) + m_points.at(p * 3 + 2)) * m_size;
            ++t_idx;
            p = m_sampleTriangles[t_idx];
            p2.x = (float(i) + m_points.at(p * 3 + 0)) * m_size;
            p2.y = (float(j) + m_points.at(p * 3 + 1)) * m_size;
            p2.z = (float(k) + m_points.at(p * 3 + 2)) * m_size;
            m_markerArray.markers.at(2).points.push_back(p2);
            m_markerArray.markers.at(2).points.push_back(p1);
            m_markerArray.markers.at(2).points.push_back(p0);

            ++t_idx;
            p = m_sampleTriangles[t_idx];
          }
        }

    m_markerPublisher->publish(m_markerArray);
  };

  void setSize(const float size) { m_size = size; };

  inline uint32_t cube_ijk2idx(const uint32_t i, const uint32_t j, const uint32_t k) const {
    return i * m_cubeCount.y * m_cubeCount.z + j * m_cubeCount.z + k;
  };

  inline uint32_t node_ijk2idx(const uint32_t i, const uint32_t j, const uint32_t k) const {
    return i * m_nodeCount.y * m_cubeCount.z + j * m_cubeCount.z + k;
  };

  void cube_idx2ijk(const uint32_t idx, Vec3<uint32_t>* out) const {
    out->z = idx % m_cubeCount.z;
    out->y = ((idx - out->z) / m_cubeCount.z) % m_cubeCount.y;
    out->x = (idx - out->z - out->y * m_cubeCount.y) % (m_cubeCount.y * m_cubeCount.z);
  };

  void node_idx2ijk(const uint32_t idx, Vec3<uint32_t>* out) const {
    out->z = idx % m_nodeCount.z;
    out->y = ((idx - out->z) / m_nodeCount.z) % m_nodeCount.y;
    out->x = (idx - out->z - out->y * m_nodeCount.y) % (m_nodeCount.y * m_nodeCount.z);
  };

 private:
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;

  double m_startTime;
  float m_size = 1.0f;
  Vec3<uint32_t> m_cubeCount = Vec3<uint32_t>(0, 0, 0);
  Vec3<uint32_t> m_nodeCount = Vec3<uint32_t>(0, 0, 0);
  thrust::device_vector<uint8_t> d_groundCubes;
  thrust::device_vector<uint8_t> d_bucketCubes;
  thrust::host_vector<uint8_t> h_groundCubes;
  thrust::host_vector<uint8_t> h_bucketCubes;
  thrust::device_vector<bool> d_groundNodes;
  thrust::device_vector<bool> d_bucketNodes;
  thrust::device_vector<float> d_groundHeights;
  thrust::device_vector<float> d_groundVolumes;
  thrust::device_vector<float> d_bucketVolumes;
  float m_totalGroundVolume = 0.0f;
  float m_totalBucketVolume = 0.0f;
  visualization_msgs::msg::MarkerArray m_markerArray;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr m_bucketSubsciber;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_markerPublisher;

  int32_t count = 100;

  const thrust::device_vector<float> d_sampleVolumes = {
      0.00000000, 0.02083333, 0.02083333, 0.12500000, 0.02083333, 0.04166667, 0.12500000, 0.35416667, 0.02083333, 0.12500000, 0.04166667, 0.35416667,
      0.12500000, 0.35416667, 0.35416667, 0.50000000, 0.02083333, 0.12500000, 0.04166667, 0.35416667, 0.04166667, 0.14583333, 0.14583333, 0.50000000,
      0.04166667, 0.35416667, 0.06250000, 0.50000000, 0.14583333, 0.50000000, 0.37500000, 0.64583333, 0.02083333, 0.04166667, 0.12500000, 0.35416667,
      0.04166667, 0.06250000, 0.35416667, 0.50000000, 0.04166667, 0.14583333, 0.14583333, 0.50000000, 0.14583333, 0.37500000, 0.50000000, 0.64583333,
      0.12500000, 0.35416667, 0.35416667, 0.50000000, 0.14583333, 0.37500000, 0.50000000, 0.64583333, 0.14583333, 0.50000000, 0.37500000, 0.64583333,
      0.25000000, 0.85416667, 0.85416667, 0.87500000, 0.02083333, 0.04166667, 0.04166667, 0.14583333, 0.12500000, 0.14583333, 0.35416667, 0.50000000,
      0.04166667, 0.14583333, 0.06250000, 0.37500000, 0.35416667, 0.50000000, 0.50000000, 0.64583333, 0.04166667, 0.14583333, 0.06250000, 0.37500000,
      0.14583333, 0.25000000, 0.37500000, 0.85416667, 0.06250000, 0.37500000, 0.08333333, 0.93750000, 0.37500000, 0.85416667, 0.93750000, 0.95833333,
      0.12500000, 0.14583333, 0.35416667, 0.50000000, 0.35416667, 0.37500000, 0.50000000, 0.64583333, 0.14583333, 0.25000000, 0.37500000, 0.85416667,
      0.50000000, 0.85416667, 0.64583333, 0.87500000, 0.35416667, 0.50000000, 0.50000000, 0.64583333, 0.50000000, 0.85416667, 0.64583333, 0.87500000,
      0.37500000, 0.85416667, 0.93750000, 0.95833333, 0.85416667, 0.95833333, 0.95833333, 0.97916667, 0.02083333, 0.04166667, 0.04166667, 0.14583333,
      0.04166667, 0.06250000, 0.14583333, 0.37500000, 0.12500000, 0.35416667, 0.14583333, 0.50000000, 0.35416667, 0.50000000, 0.50000000, 0.64583333,
      0.12500000, 0.35416667, 0.14583333, 0.50000000, 0.14583333, 0.37500000, 0.25000000, 0.85416667, 0.35416667, 0.50000000, 0.37500000, 0.64583333,
      0.50000000, 0.64583333, 0.85416667, 0.87500000, 0.04166667, 0.06250000, 0.14583333, 0.37500000, 0.06250000, 0.08333333, 0.37500000, 0.93750000,
      0.14583333, 0.37500000, 0.25000000, 0.85416667, 0.37500000, 0.93750000, 0.85416667, 0.95833333, 0.35416667, 0.50000000, 0.50000000, 0.64583333,
      0.37500000, 0.93750000, 0.85416667, 0.95833333, 0.50000000, 0.64583333, 0.85416667, 0.87500000, 0.85416667, 0.95833333, 0.95833333, 0.97916667,
      0.12500000, 0.14583333, 0.14583333, 0.25000000, 0.35416667, 0.37500000, 0.50000000, 0.85416667, 0.35416667, 0.50000000, 0.37500000, 0.85416667,
      0.50000000, 0.64583333, 0.64583333, 0.87500000, 0.35416667, 0.50000000, 0.37500000, 0.85416667, 0.50000000, 0.85416667, 0.85416667, 0.95833333,
      0.50000000, 0.64583333, 0.93750000, 0.95833333, 0.64583333, 0.87500000, 0.95833333, 0.97916667, 0.35416667, 0.37500000, 0.50000000, 0.85416667,
      0.50000000, 0.93750000, 0.64583333, 0.95833333, 0.50000000, 0.85416667, 0.85416667, 0.95833333, 0.64583333, 0.95833333, 0.87500000, 0.97916667,
      0.50000000, 0.64583333, 0.64583333, 0.87500000, 0.64583333, 0.95833333, 0.87500000, 0.97916667, 0.64583333, 0.87500000, 0.95833333, 0.97916667,
      0.87500000, 0.97916667, 0.97916667, 1.00000000};

  const std::array<float, 12 * 3> m_points = {0.5f, 0.0f, 0.0f, 1.0f, 0.5f, 0.0f, 0.5f, 1.0f, 0.0f, 0.0f, 0.5f, 0.0f,
                                              0.5f, 0.0f, 1.0f, 1.0f, 0.5f, 1.0f, 0.5f, 1.0f, 1.0f, 0.0f, 0.5f, 1.0f,
                                              0.0f, 0.0f, 0.5f, 1.0f, 0.0f, 0.5f, 1.0f, 1.0f, 0.5f, 0.0f, 1.0f, 0.5f};

  const std::array<int8_t, 256 * 16> m_sampleTriangles = {
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  8,  3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  1,  9,  -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  8,  3,  9,  8,  1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  2,  10, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, 0,  8,  3,  1,  2,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 9,  2,  10, 0,  2,  9,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 2,  8,  3,  2,  10, 8,  10, 9,  8,  -1, -1, -1, -1, -1, -1, -1, 3,  11, 2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      0,  11, 2,  8,  11, 0,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  9,  0,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  11, 2,  1,
      9,  11, 9,  8,  11, -1, -1, -1, -1, -1, -1, -1, 3,  10, 1,  11, 10, 3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  10, 1,  0,  8,  10, 8,  11,
      10, -1, -1, -1, -1, -1, -1, -1, 3,  9,  0,  3,  11, 9,  11, 10, 9,  -1, -1, -1, -1, -1, -1, -1, 9,  8,  10, 10, 8,  11, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 4,  7,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4,  3,  0,  7,  3,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      0,  1,  9,  8,  4,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4,  1,  9,  4,  7,  1,  7,  3,  1,  -1, -1, -1, -1, -1, -1, -1, 1,  2,  10, 8,
      4,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3,  4,  7,  3,  0,  4,  1,  2,  10, -1, -1, -1, -1, -1, -1, -1, 9,  2,  10, 9,  0,  2,  8,  4,
      7,  -1, -1, -1, -1, -1, -1, -1, 2,  10, 9,  2,  9,  7,  2,  7,  3,  7,  9,  4,  -1, -1, -1, -1, 8,  4,  7,  3,  11, 2,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 11, 4,  7,  11, 2,  4,  2,  0,  4,  -1, -1, -1, -1, -1, -1, -1, 9,  0,  1,  8,  4,  7,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1,
      4,  7,  11, 9,  4,  11, 9,  11, 2,  9,  2,  1,  -1, -1, -1, -1, 3,  10, 1,  3,  11, 10, 7,  8,  4,  -1, -1, -1, -1, -1, -1, -1, 1,  11, 10, 1,
      4,  11, 1,  0,  4,  7,  11, 4,  -1, -1, -1, -1, 4,  7,  8,  9,  0,  11, 9,  11, 10, 11, 0,  3,  -1, -1, -1, -1, 4,  7,  11, 4,  11, 9,  9,  11,
      10, -1, -1, -1, -1, -1, -1, -1, 9,  5,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 9,  5,  4,  0,  8,  3,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 0,  5,  4,  1,  5,  0,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 8,  5,  4,  8,  3,  5,  3,  1,  5,  -1, -1, -1, -1, -1, -1, -1,
      1,  2,  10, 9,  5,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3,  0,  8,  1,  2,  10, 4,  9,  5,  -1, -1, -1, -1, -1, -1, -1, 5,  2,  10, 5,
      4,  2,  4,  0,  2,  -1, -1, -1, -1, -1, -1, -1, 2,  10, 5,  3,  2,  5,  3,  5,  4,  3,  4,  8,  -1, -1, -1, -1, 9,  5,  4,  2,  3,  11, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, 0,  11, 2,  0,  8,  11, 4,  9,  5,  -1, -1, -1, -1, -1, -1, -1, 0,  5,  4,  0,  1,  5,  2,  3,  11, -1, -1, -1,
      -1, -1, -1, -1, 2,  1,  5,  2,  5,  8,  2,  8,  11, 4,  8,  5,  -1, -1, -1, -1, 10, 3,  11, 10, 1,  3,  9,  5,  4,  -1, -1, -1, -1, -1, -1, -1,
      4,  9,  5,  0,  8,  1,  8,  10, 1,  8,  11, 10, -1, -1, -1, -1, 5,  4,  0,  5,  0,  11, 5,  11, 10, 11, 0,  3,  -1, -1, -1, -1, 5,  4,  8,  5,
      8,  10, 10, 8,  11, -1, -1, -1, -1, -1, -1, -1, 9,  7,  8,  5,  7,  9,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 9,  3,  0,  9,  5,  3,  5,  7,
      3,  -1, -1, -1, -1, -1, -1, -1, 0,  7,  8,  0,  1,  7,  1,  5,  7,  -1, -1, -1, -1, -1, -1, -1, 1,  5,  3,  3,  5,  7,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 9,  7,  8,  9,  5,  7,  10, 1,  2,  -1, -1, -1, -1, -1, -1, -1, 10, 1,  2,  9,  5,  0,  5,  3,  0,  5,  7,  3,  -1, -1, -1, -1,
      8,  0,  2,  8,  2,  5,  8,  5,  7,  10, 5,  2,  -1, -1, -1, -1, 2,  10, 5,  2,  5,  3,  3,  5,  7,  -1, -1, -1, -1, -1, -1, -1, 7,  9,  5,  7,
      8,  9,  3,  11, 2,  -1, -1, -1, -1, -1, -1, -1, 9,  5,  7,  9,  7,  2,  9,  2,  0,  2,  7,  11, -1, -1, -1, -1, 2,  3,  11, 0,  1,  8,  1,  7,
      8,  1,  5,  7,  -1, -1, -1, -1, 11, 2,  1,  11, 1,  7,  7,  1,  5,  -1, -1, -1, -1, -1, -1, -1, 9,  5,  8,  8,  5,  7,  10, 1,  3,  10, 3,  11,
      -1, -1, -1, -1, 5,  7,  0,  5,  0,  9,  7,  11, 0,  1,  0,  10, 11, 10, 0,  -1, 11, 10, 0,  11, 0,  3,  10, 5,  0,  8,  0,  7,  5,  7,  0,  -1,
      11, 10, 5,  7,  11, 5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 10, 6,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  8,  3,  5,
      10, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 9,  0,  1,  5,  10, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  8,  3,  1,  9,  8,  5,  10,
      6,  -1, -1, -1, -1, -1, -1, -1, 1,  6,  5,  2,  6,  1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  6,  5,  1,  2,  6,  3,  0,  8,  -1, -1, -1,
      -1, -1, -1, -1, 9,  6,  5,  9,  0,  6,  0,  2,  6,  -1, -1, -1, -1, -1, -1, -1, 5,  9,  8,  5,  8,  2,  5,  2,  6,  3,  2,  8,  -1, -1, -1, -1,
      2,  3,  11, 10, 6,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 11, 0,  8,  11, 2,  0,  10, 6,  5,  -1, -1, -1, -1, -1, -1, -1, 0,  1,  9,  2,
      3,  11, 5,  10, 6,  -1, -1, -1, -1, -1, -1, -1, 5,  10, 6,  1,  9,  2,  9,  11, 2,  9,  8,  11, -1, -1, -1, -1, 6,  3,  11, 6,  5,  3,  5,  1,
      3,  -1, -1, -1, -1, -1, -1, -1, 0,  8,  11, 0,  11, 5,  0,  5,  1,  5,  11, 6,  -1, -1, -1, -1, 3,  11, 6,  0,  3,  6,  0,  6,  5,  0,  5,  9,
      -1, -1, -1, -1, 6,  5,  9,  6,  9,  11, 11, 9,  8,  -1, -1, -1, -1, -1, -1, -1, 5,  10, 6,  4,  7,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      4,  3,  0,  4,  7,  3,  6,  5,  10, -1, -1, -1, -1, -1, -1, -1, 1,  9,  0,  5,  10, 6,  8,  4,  7,  -1, -1, -1, -1, -1, -1, -1, 10, 6,  5,  1,
      9,  7,  1,  7,  3,  7,  9,  4,  -1, -1, -1, -1, 6,  1,  2,  6,  5,  1,  4,  7,  8,  -1, -1, -1, -1, -1, -1, -1, 1,  2,  5,  5,  2,  6,  3,  0,
      4,  3,  4,  7,  -1, -1, -1, -1, 8,  4,  7,  9,  0,  5,  0,  6,  5,  0,  2,  6,  -1, -1, -1, -1, 7,  3,  9,  7,  9,  4,  3,  2,  9,  5,  9,  6,
      2,  6,  9,  -1, 3,  11, 2,  7,  8,  4,  10, 6,  5,  -1, -1, -1, -1, -1, -1, -1, 5,  10, 6,  4,  7,  2,  4,  2,  0,  2,  7,  11, -1, -1, -1, -1,
      0,  1,  9,  4,  7,  8,  2,  3,  11, 5,  10, 6,  -1, -1, -1, -1, 9,  2,  1,  9,  11, 2,  9,  4,  11, 7,  11, 4,  5,  10, 6,  -1, 8,  4,  7,  3,
      11, 5,  3,  5,  1,  5,  11, 6,  -1, -1, -1, -1, 5,  1,  11, 5,  11, 6,  1,  0,  11, 7,  11, 4,  0,  4,  11, -1, 0,  5,  9,  0,  6,  5,  0,  3,
      6,  11, 6,  3,  8,  4,  7,  -1, 6,  5,  9,  6,  9,  11, 4,  7,  9,  7,  11, 9,  -1, -1, -1, -1, 10, 4,  9,  6,  4,  10, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 4,  10, 6,  4,  9,  10, 0,  8,  3,  -1, -1, -1, -1, -1, -1, -1, 10, 0,  1,  10, 6,  0,  6,  4,  0,  -1, -1, -1, -1, -1, -1, -1,
      8,  3,  1,  8,  1,  6,  8,  6,  4,  6,  1,  10, -1, -1, -1, -1, 1,  4,  9,  1,  2,  4,  2,  6,  4,  -1, -1, -1, -1, -1, -1, -1, 3,  0,  8,  1,
      2,  9,  2,  4,  9,  2,  6,  4,  -1, -1, -1, -1, 0,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 8,  3,  2,  8,  2,  4,  4,  2,
      6,  -1, -1, -1, -1, -1, -1, -1, 10, 4,  9,  10, 6,  4,  11, 2,  3,  -1, -1, -1, -1, -1, -1, -1, 0,  8,  2,  2,  8,  11, 4,  9,  10, 4,  10, 6,
      -1, -1, -1, -1, 3,  11, 2,  0,  1,  6,  0,  6,  4,  6,  1,  10, -1, -1, -1, -1, 6,  4,  1,  6,  1,  10, 4,  8,  1,  2,  1,  11, 8,  11, 1,  -1,
      9,  6,  4,  9,  3,  6,  9,  1,  3,  11, 6,  3,  -1, -1, -1, -1, 8,  11, 1,  8,  1,  0,  11, 6,  1,  9,  1,  4,  6,  4,  1,  -1, 3,  11, 6,  3,
      6,  0,  0,  6,  4,  -1, -1, -1, -1, -1, -1, -1, 6,  4,  8,  11, 6,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 7,  10, 6,  7,  8,  10, 8,  9,
      10, -1, -1, -1, -1, -1, -1, -1, 0,  7,  3,  0,  10, 7,  0,  9,  10, 6,  7,  10, -1, -1, -1, -1, 10, 6,  7,  1,  10, 7,  1,  7,  8,  1,  8,  0,
      -1, -1, -1, -1, 10, 6,  7,  10, 7,  1,  1,  7,  3,  -1, -1, -1, -1, -1, -1, -1, 1,  2,  6,  1,  6,  8,  1,  8,  9,  8,  6,  7,  -1, -1, -1, -1,
      2,  6,  9,  2,  9,  1,  6,  7,  9,  0,  9,  3,  7,  3,  9,  -1, 7,  8,  0,  7,  0,  6,  6,  0,  2,  -1, -1, -1, -1, -1, -1, -1, 7,  3,  2,  6,
      7,  2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2,  3,  11, 10, 6,  8,  10, 8,  9,  8,  6,  7,  -1, -1, -1, -1, 2,  0,  7,  2,  7,  11, 0,  9,
      7,  6,  7,  10, 9,  10, 7,  -1, 1,  8,  0,  1,  7,  8,  1,  10, 7,  6,  7,  10, 2,  3,  11, -1, 11, 2,  1,  11, 1,  7,  10, 6,  1,  6,  7,  1,
      -1, -1, -1, -1, 8,  9,  6,  8,  6,  7,  9,  1,  6,  11, 6,  3,  1,  3,  6,  -1, 0,  9,  1,  11, 6,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      7,  8,  0,  7,  0,  6,  3,  11, 0,  11, 6,  0,  -1, -1, -1, -1, 7,  11, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 7,  6,  11, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3,  0,  8,  11, 7,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  1,  9,  11, 7,  6,  -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, 8,  1,  9,  8,  3,  1,  11, 7,  6,  -1, -1, -1, -1, -1, -1, -1, 10, 1,  2,  6,  11, 7,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 1,  2,  10, 3,  0,  8,  6,  11, 7,  -1, -1, -1, -1, -1, -1, -1, 2,  9,  0,  2,  10, 9,  6,  11, 7,  -1, -1, -1, -1, -1, -1, -1,
      6,  11, 7,  2,  10, 3,  10, 8,  3,  10, 9,  8,  -1, -1, -1, -1, 7,  2,  3,  6,  2,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 7,  0,  8,  7,
      6,  0,  6,  2,  0,  -1, -1, -1, -1, -1, -1, -1, 2,  7,  6,  2,  3,  7,  0,  1,  9,  -1, -1, -1, -1, -1, -1, -1, 1,  6,  2,  1,  8,  6,  1,  9,
      8,  8,  7,  6,  -1, -1, -1, -1, 10, 7,  6,  10, 1,  7,  1,  3,  7,  -1, -1, -1, -1, -1, -1, -1, 10, 7,  6,  1,  7,  10, 1,  8,  7,  1,  0,  8,
      -1, -1, -1, -1, 0,  3,  7,  0,  7,  10, 0,  10, 9,  6,  10, 7,  -1, -1, -1, -1, 7,  6,  10, 7,  10, 8,  8,  10, 9,  -1, -1, -1, -1, -1, -1, -1,
      6,  8,  4,  11, 8,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3,  6,  11, 3,  0,  6,  0,  4,  6,  -1, -1, -1, -1, -1, -1, -1, 8,  6,  11, 8,
      4,  6,  9,  0,  1,  -1, -1, -1, -1, -1, -1, -1, 9,  4,  6,  9,  6,  3,  9,  3,  1,  11, 3,  6,  -1, -1, -1, -1, 6,  8,  4,  6,  11, 8,  2,  10,
      1,  -1, -1, -1, -1, -1, -1, -1, 1,  2,  10, 3,  0,  11, 0,  6,  11, 0,  4,  6,  -1, -1, -1, -1, 4,  11, 8,  4,  6,  11, 0,  2,  9,  2,  10, 9,
      -1, -1, -1, -1, 10, 9,  3,  10, 3,  2,  9,  4,  3,  11, 3,  6,  4,  6,  3,  -1, 8,  2,  3,  8,  4,  2,  4,  6,  2,  -1, -1, -1, -1, -1, -1, -1,
      0,  4,  2,  4,  6,  2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  9,  0,  2,  3,  4,  2,  4,  6,  4,  3,  8,  -1, -1, -1, -1, 1,  9,  4,  1,
      4,  2,  2,  4,  6,  -1, -1, -1, -1, -1, -1, -1, 8,  1,  3,  8,  6,  1,  8,  4,  6,  6,  10, 1,  -1, -1, -1, -1, 10, 1,  0,  10, 0,  6,  6,  0,
      4,  -1, -1, -1, -1, -1, -1, -1, 4,  6,  3,  4,  3,  8,  6,  10, 3,  0,  3,  9,  10, 9,  3,  -1, 10, 9,  4,  6,  10, 4,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 4,  9,  5,  7,  6,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  8,  3,  4,  9,  5,  11, 7,  6,  -1, -1, -1, -1, -1, -1, -1,
      5,  0,  1,  5,  4,  0,  7,  6,  11, -1, -1, -1, -1, -1, -1, -1, 11, 7,  6,  8,  3,  4,  3,  5,  4,  3,  1,  5,  -1, -1, -1, -1, 9,  5,  4,  10,
      1,  2,  7,  6,  11, -1, -1, -1, -1, -1, -1, -1, 6,  11, 7,  1,  2,  10, 0,  8,  3,  4,  9,  5,  -1, -1, -1, -1, 7,  6,  11, 5,  4,  10, 4,  2,
      10, 4,  0,  2,  -1, -1, -1, -1, 3,  4,  8,  3,  5,  4,  3,  2,  5,  10, 5,  2,  11, 7,  6,  -1, 7,  2,  3,  7,  6,  2,  5,  4,  9,  -1, -1, -1,
      -1, -1, -1, -1, 9,  5,  4,  0,  8,  6,  0,  6,  2,  6,  8,  7,  -1, -1, -1, -1, 3,  6,  2,  3,  7,  6,  1,  5,  0,  5,  4,  0,  -1, -1, -1, -1,
      6,  2,  8,  6,  8,  7,  2,  1,  8,  4,  8,  5,  1,  5,  8,  -1, 9,  5,  4,  10, 1,  6,  1,  7,  6,  1,  3,  7,  -1, -1, -1, -1, 1,  6,  10, 1,
      7,  6,  1,  0,  7,  8,  7,  0,  9,  5,  4,  -1, 4,  0,  10, 4,  10, 5,  0,  3,  10, 6,  10, 7,  3,  7,  10, -1, 7,  6,  10, 7,  10, 8,  5,  4,
      10, 4,  8,  10, -1, -1, -1, -1, 6,  9,  5,  6,  11, 9,  11, 8,  9,  -1, -1, -1, -1, -1, -1, -1, 3,  6,  11, 0,  6,  3,  0,  5,  6,  0,  9,  5,
      -1, -1, -1, -1, 0,  11, 8,  0,  5,  11, 0,  1,  5,  5,  6,  11, -1, -1, -1, -1, 6,  11, 3,  6,  3,  5,  5,  3,  1,  -1, -1, -1, -1, -1, -1, -1,
      1,  2,  10, 9,  5,  11, 9,  11, 8,  11, 5,  6,  -1, -1, -1, -1, 0,  11, 3,  0,  6,  11, 0,  9,  6,  5,  6,  9,  1,  2,  10, -1, 11, 8,  5,  11,
      5,  6,  8,  0,  5,  10, 5,  2,  0,  2,  5,  -1, 6,  11, 3,  6,  3,  5,  2,  10, 3,  10, 5,  3,  -1, -1, -1, -1, 5,  8,  9,  5,  2,  8,  5,  6,
      2,  3,  8,  2,  -1, -1, -1, -1, 9,  5,  6,  9,  6,  0,  0,  6,  2,  -1, -1, -1, -1, -1, -1, -1, 1,  5,  8,  1,  8,  0,  5,  6,  8,  3,  8,  2,
      6,  2,  8,  -1, 1,  5,  6,  2,  1,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  3,  6,  1,  6,  10, 3,  8,  6,  5,  6,  9,  8,  9,  6,  -1,
      10, 1,  0,  10, 0,  6,  9,  5,  0,  5,  6,  0,  -1, -1, -1, -1, 0,  3,  8,  5,  6,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 10, 5,  6,  -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 11, 5,  10, 7,  5,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 11, 5,  10, 11, 7,  5,  8,  3,
      0,  -1, -1, -1, -1, -1, -1, -1, 5,  11, 7,  5,  10, 11, 1,  9,  0,  -1, -1, -1, -1, -1, -1, -1, 10, 7,  5,  10, 11, 7,  9,  8,  1,  8,  3,  1,
      -1, -1, -1, -1, 11, 1,  2,  11, 7,  1,  7,  5,  1,  -1, -1, -1, -1, -1, -1, -1, 0,  8,  3,  1,  2,  7,  1,  7,  5,  7,  2,  11, -1, -1, -1, -1,
      9,  7,  5,  9,  2,  7,  9,  0,  2,  2,  11, 7,  -1, -1, -1, -1, 7,  5,  2,  7,  2,  11, 5,  9,  2,  3,  2,  8,  9,  8,  2,  -1, 2,  5,  10, 2,
      3,  5,  3,  7,  5,  -1, -1, -1, -1, -1, -1, -1, 8,  2,  0,  8,  5,  2,  8,  7,  5,  10, 2,  5,  -1, -1, -1, -1, 9,  0,  1,  5,  10, 3,  5,  3,
      7,  3,  10, 2,  -1, -1, -1, -1, 9,  8,  2,  9,  2,  1,  8,  7,  2,  10, 2,  5,  7,  5,  2,  -1, 1,  3,  5,  3,  7,  5,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 0,  8,  7,  0,  7,  1,  1,  7,  5,  -1, -1, -1, -1, -1, -1, -1, 9,  0,  3,  9,  3,  5,  5,  3,  7,  -1, -1, -1, -1, -1, -1, -1,
      9,  8,  7,  5,  9,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 5,  8,  4,  5,  10, 8,  10, 11, 8,  -1, -1, -1, -1, -1, -1, -1, 5,  0,  4,  5,
      11, 0,  5,  10, 11, 11, 3,  0,  -1, -1, -1, -1, 0,  1,  9,  8,  4,  10, 8,  10, 11, 10, 4,  5,  -1, -1, -1, -1, 10, 11, 4,  10, 4,  5,  11, 3,
      4,  9,  4,  1,  3,  1,  4,  -1, 2,  5,  1,  2,  8,  5,  2,  11, 8,  4,  5,  8,  -1, -1, -1, -1, 0,  4,  11, 0,  11, 3,  4,  5,  11, 2,  11, 1,
      5,  1,  11, -1, 0,  2,  5,  0,  5,  9,  2,  11, 5,  4,  5,  8,  11, 8,  5,  -1, 9,  4,  5,  2,  11, 3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      2,  5,  10, 3,  5,  2,  3,  4,  5,  3,  8,  4,  -1, -1, -1, -1, 5,  10, 2,  5,  2,  4,  4,  2,  0,  -1, -1, -1, -1, -1, -1, -1, 3,  10, 2,  3,
      5,  10, 3,  8,  5,  4,  5,  8,  0,  1,  9,  -1, 5,  10, 2,  5,  2,  4,  1,  9,  2,  9,  4,  2,  -1, -1, -1, -1, 8,  4,  5,  8,  5,  3,  3,  5,
      1,  -1, -1, -1, -1, -1, -1, -1, 0,  4,  5,  1,  0,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 8,  4,  5,  8,  5,  3,  9,  0,  5,  0,  3,  5,
      -1, -1, -1, -1, 9,  4,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4,  11, 7,  4,  9,  11, 9,  10, 11, -1, -1, -1, -1, -1, -1, -1,
      0,  8,  3,  4,  9,  7,  9,  11, 7,  9,  10, 11, -1, -1, -1, -1, 1,  10, 11, 1,  11, 4,  1,  4,  0,  7,  4,  11, -1, -1, -1, -1, 3,  1,  4,  3,
      4,  8,  1,  10, 4,  7,  4,  11, 10, 11, 4,  -1, 4,  11, 7,  9,  11, 4,  9,  2,  11, 9,  1,  2,  -1, -1, -1, -1, 9,  7,  4,  9,  11, 7,  9,  1,
      11, 2,  11, 1,  0,  8,  3,  -1, 11, 7,  4,  11, 4,  2,  2,  4,  0,  -1, -1, -1, -1, -1, -1, -1, 11, 7,  4,  11, 4,  2,  8,  3,  4,  3,  2,  4,
      -1, -1, -1, -1, 2,  9,  10, 2,  7,  9,  2,  3,  7,  7,  4,  9,  -1, -1, -1, -1, 9,  10, 7,  9,  7,  4,  10, 2,  7,  8,  7,  0,  2,  0,  7,  -1,
      3,  7,  10, 3,  10, 2,  7,  4,  10, 1,  10, 0,  4,  0,  10, -1, 1,  10, 2,  8,  7,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4,  9,  1,  4,
      1,  7,  7,  1,  3,  -1, -1, -1, -1, -1, -1, -1, 4,  9,  1,  4,  1,  7,  0,  8,  1,  8,  7,  1,  -1, -1, -1, -1, 4,  0,  3,  7,  4,  3,  -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, 4,  8,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 9,  10, 8,  10, 11, 8,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 3,  0,  9,  3,  9,  11, 11, 9,  10, -1, -1, -1, -1, -1, -1, -1, 0,  1,  10, 0,  10, 8,  8,  10, 11, -1, -1, -1, -1, -1, -1, -1,
      3,  1,  10, 11, 3,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1,  2,  11, 1,  11, 9,  9,  11, 8,  -1, -1, -1, -1, -1, -1, -1, 3,  0,  9,  3,
      9,  11, 1,  2,  9,  2,  11, 9,  -1, -1, -1, -1, 0,  2,  11, 8,  0,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3,  2,  11, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, 2,  3,  8,  2,  8,  10, 10, 8,  9,  -1, -1, -1, -1, -1, -1, -1, 9,  10, 2,  0,  9,  2,  -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, 2,  3,  8,  2,  8,  10, 0,  1,  8,  1,  10, 8,  -1, -1, -1, -1, 1,  10, 2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      1,  3,  8,  9,  1,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  9,  1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  3,  8,  -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarchingCubes>(400, 400, 100, 0.01));
  rclcpp::shutdown();
  return 0;
}
