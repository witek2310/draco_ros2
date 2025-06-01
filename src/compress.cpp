#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <draco_cpp/msg/compressed_point_cloud_draco.h> // Replace with your actual package name

#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/core/encoder_buffer.h>

#include <chrono>

class DracoCompressorNode : public rclcpp::Node {
public:
  DracoCompressorNode()
  : Node("draco_compressor_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/input_pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&DracoCompressorNode::pointcloud_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<draco::msg::CompressedPointCloud2>(
      "/compressed_pointcloud", 10);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert ROS PointCloud2 to Draco PointCloud
    draco::PointCloudBuilder builder;
    builder.Start(msg->width * msg->height);

    // Add POSITION attribute
    int pos_att_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z) {
      float position[3] = {*iter_x, *iter_y, *iter_z};
      builder.SetAttributeValueForPoint(pos_att_id, i, &position[0]);
    }

    std::unique_ptr<draco::PointCloud> draco_pc = builder.Finalize(false);

    // Set encoder options
    draco::Encoder encoder;
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14); // 14 bits quantization
    encoder.SetSpeedOptions(5, 5); // Encoding and decoding speed

    draco::EncoderBuffer buffer;

    // Measure compression time
    auto start = std::chrono::high_resolution_clock::now();
    const draco::Status status = encoder.EncodePointCloudToBuffer(*draco_pc, &buffer);
    auto end = std::chrono::high_resolution_clock::now();

    if (!status.ok()) {
      RCLCPP_ERROR(this->get_logger(), "Draco compression failed: %s", status.error_msg());
      return;
    }

    std::chrono::duration<double, std::milli> compression_time = end - start;

    // Calculate compression ratio
    size_t original_size = msg->data.size();
    size_t compressed_size = buffer.size();
    double compression_ratio = static_cast<double>(original_size) / compressed_size;

    RCLCPP_INFO(this->get_logger(),
                "Compression Time: %.2f ms | Original Size: %zu bytes | Compressed Size: %zu bytes | Compression Ratio: %.2f",
                compression_time.count(), original_size, compressed_size, compression_ratio);

    // Publish compressed point cloud
    draco::msg::CompressedPointCloud2 compressed_msg;
    compressed_msg.header = msg->header;
    compressed_msg.format = "draco";
    compressed_msg.data = std::vector<uint8_t>(buffer.data(), buffer.data() + buffer.size());

    publisher_->publish(compressed_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<draco::msg::CompressedPointCloud2>::SharedPtr publisher_;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DracoDecompressorNode>());
  rclcpp::shutdown();
  return 0;
}