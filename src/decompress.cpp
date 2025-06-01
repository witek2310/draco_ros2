#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <draco/msg/compressed_point_cloud2.hpp> // Replace with your actual package name

#include <draco/compression/decode.h>
#include <draco/core/decoder_buffer.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <chrono>

class DracoDecompressorNode : public rclcpp::Node {
public:
  DracoDecompressorNode()
  : Node("draco_decompressor_node") {
    subscription_ = this->create_subscription<draco::msg::CompressedPointCloud2>(
      "/compressed_pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&DracoDecompressorNode::compressed_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/decompressed_pointcloud", rclcpp::SensorDataQoS());
  }

private:
  void compressed_callback(const draco::msg::CompressedPointCloud2::SharedPtr msg) {
    if (msg->format != "draco") {
      RCLCPP_WARN(this->get_logger(), "Unsupported compression format: %s", msg->format.c_str());
      return;
    }

    // Initialize Draco decoder buffer
    draco::DecoderBuffer buffer;
    buffer.Init(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());

    // Decode the point cloud
    auto start = std::chrono::high_resolution_clock::now();
    draco::Decoder decoder;
    auto status_or_pc = decoder.DecodePointCloudFromBuffer(&buffer);
    auto end = std::chrono::high_resolution_clock::now();

    if (!status_or_pc.ok()) {
      RCLCPP_ERROR(this->get_logger(), "Draco decompression failed: %s", status_or_pc.status().error_msg());
      return;
    }

    std::chrono::duration<double, std::milli> decompression_time = end - start;
    RCLCPP_INFO(this->get_logger(), "Decompression Time: %.2f ms", decompression_time.count());

    std::unique_ptr<draco::PointCloud> draco_pc = std::move(status_or_pc).value();

    // Retrieve POSITION attribute
    const draco::PointAttribute* pos_attribute = draco_pc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (pos_attribute == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "No POSITION attribute found in Draco point cloud.");
      return;
    }

    // Prepare PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_msg;
    output_msg.header = msg->header;
    output_msg.height = 1;
    output_msg.width = draco_pc->num_points();
    output_msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(output_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(draco_pc->num_points());

    sensor_msgs::PointCloud2Iterator<float> iter_x(output_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output_msg, "z");

    for (draco::PointIndex i(0); i < draco_pc->num_points(); ++i, ++iter_x, ++iter_y, ++iter_z) {
      float pos[3];
      if (!pos_attribute->ConvertValue<float, 3>(pos_attribute->mapped_index(i), pos)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert position attribute at point %d.", i.value());
        return;
      }
      *iter_x = pos[0];
      *iter_y = pos[1];
      *iter_z = pos[2];
    }

    // Log decompressed size
    size_t decompressed_size = output_msg.data.size();
    RCLCPP_INFO(this->get_logger(), "Decompressed PointCloud2 size: %zu bytes", decompressed_size);

    // Publish the decompressed point cloud
    publisher_->publish(output_msg);
  }

  rclcpp::Subscription<draco::msg::CompressedPointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};
