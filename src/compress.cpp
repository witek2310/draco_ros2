#include <fstream>
#include <filesystem>


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


#include <draco_ros2/msg/compressed_point_cloud_draco.hpp> // Replace with your actual package name

#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/core/encoder_buffer.h>

#include <chrono>

class DracoCompressorNode : public rclcpp::Node {
public:
  DracoCompressorNode()
  : Node("draco_compressor_node") {
    this->declare_parameter<int>("quantization_bits", 14);
    this->declare_parameter<int>("encoding_speed", 10);
    this->declare_parameter<int>("decoding_speed", 10);
    this->declare_parameter<std::string>("csv_folder_path", "/tmp");
    this->declare_parameter<std::string>("pointcloud_topic", "/velodyne/velodyne_points");

    _encoding_speed = this->get_parameter("encoding_speed").as_int();
    _decoding_speed = this->get_parameter("decoding_speed").as_int();
    _quantization_bits = this->get_parameter("quantization_bits").as_int();

    std::string pointcloud_topic;
    this->get_parameter("pointcloud_topic", pointcloud_topic);
    RCLCPP_INFO(this->get_logger(), "Subscribing to point cloud topic: %s", pointcloud_topic.c_str());

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&DracoCompressorNode::pointcloud_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<draco_ros2::msg::CompressedPointCloudDraco>(
      "/compressed_pointcloud", 10);


    std::string folder_path;
    this->get_parameter("csv_folder_path", folder_path);
    csv_file_path_ = folder_path+ "/compress.csv";

    // saving data to a csv file
    if (!std::filesystem::exists(csv_file_path_)) {
      std::filesystem::create_directories(std::filesystem::path(csv_file_path_).parent_path()); 
      std::ofstream file(csv_file_path_);
      if (file.is_open()) {
        file << "time,points_number,point_cloud_size,compresion_time,size_after_compresion,quantization_bits,encoding_speed,decoding_speed\n";  // Header row
        file.close();
      }
    }

        RCLCPP_INFO(this->get_logger(),
                "Started node");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert ROS PointCloud2 to Draco PointCloud

    RCLCPP_INFO(this->get_logger(),"pc recived");

    auto start = std::chrono::high_resolution_clock::now();

    draco::PointCloudBuilder builder;
    builder.Start(msg->width * msg->height);
      
    // Add POSITION attribute
    int pos_att_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z) {
      float position[3] = {*iter_x, *iter_y, *iter_z};
      builder.SetAttributeValueForPoint(pos_att_id, draco::PointIndex(i), &position[0]);
    }

    std::unique_ptr<draco::PointCloud> draco_pc = builder.Finalize(false);

    // Set encoder options
    draco::Encoder encoder;
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, _quantization_bits); // 14 bits quantization
    encoder.SetSpeedOptions(_encoding_speed, _decoding_speed); // Encoding and decoding speed

    draco::EncoderBuffer buffer;

    // Measure compression time
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
    draco_ros2::msg::CompressedPointCloudDraco compressed_msg;
    compressed_msg.header = msg->header;
    compressed_msg.data = std::vector<uint8_t>(buffer.data(), buffer.data() + buffer.size());

    publisher_->publish(compressed_msg);


    std::ofstream ofs(csv_file_path_, std::ios_base::app);
    if (!ofs) {
      std::filesystem::create_directories(std::filesystem::path(csv_file_path_).parent_path()); 
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", csv_file_path_.c_str());
      return;
    }

    int64_t t_sec = msg->header.stamp.sec;
    int64_t t_nsec = msg->header.stamp.nanosec;
    double time = static_cast<double>(t_sec) + 1e-9 * static_cast<double>(t_nsec);

    ofs << time<<"," << msg->width * msg->height <<"," <<msg->row_step * msg->height << ", " << compression_time.count() << ", " << buffer.size() <<"," << _quantization_bits <<","<< _encoding_speed <<"," << _decoding_speed<<"\n";
    ofs.close();

  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<draco_ros2::msg::CompressedPointCloudDraco>::SharedPtr publisher_;
  int _decoding_speed, _encoding_speed, _quantization_bits;
  std::string csv_file_path_;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DracoCompressorNode>());
  rclcpp::shutdown();
  return 0;
}