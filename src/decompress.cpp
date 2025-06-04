#include <fstream>
#include <filesystem>


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


#include <draco_ros2/msg/compressed_point_cloud_draco.hpp>// Replace with your actual package name

#include <draco/compression/decode.h>
#include <draco/core/decoder_buffer.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <chrono>

class DracoDecompressorNode : public rclcpp::Node {
public:
  DracoDecompressorNode()
  : Node("draco_decompressor_node") {
    this->declare_parameter<std::string>("csv_folder_path", "/tmp");
    subscription_ = this->create_subscription<draco_ros2::msg::CompressedPointCloudDraco>(
      "/compressed_pointcloud", 10,
      std::bind(&DracoDecompressorNode::compressed_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/decompressed_pointcloud_draco", 10);


    std::string folder_path;
    this->get_parameter("csv_folder_path", folder_path);
    csv_file_path_ = folder_path + "/decompress_draco.csv";

    // saving data to a csv file
    if (!std::filesystem::exists(csv_file_path_)) {
      std::filesystem::create_directories(std::filesystem::path(csv_file_path_).parent_path()); 
      std::ofstream file(csv_file_path_);
      if (file.is_open()) {
        file << "time_stamp,points_number_after_decompression,decompresion_time,size_before_decompression\n";
        file.close();
      }
    }
  }

private:
  void compressed_callback(const draco_ros2::msg::CompressedPointCloudDraco::SharedPtr msg) {

    // Initialize Draco decoder buffer
    draco::DecoderBuffer buffer;
    buffer.Init(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());

    // Decode the point cloud
    auto start = std::chrono::high_resolution_clock::now();
    draco::Decoder decoder;
    auto status_or_pc = decoder.DecodePointCloudFromBuffer(&buffer);


    if (!status_or_pc.ok()) {
      RCLCPP_ERROR(this->get_logger(), "Draco decompression failed: %s", status_or_pc.status().error_msg());
      return;
    }

    
    

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

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> decompression_time = end - start;

    RCLCPP_INFO(this->get_logger(), "Decompression Time: %.2f ms", decompression_time.count());

    // Publish the decompressed point cloud
    publisher_->publish(output_msg);

    int64_t t_sec = msg->header.stamp.sec;
    int64_t t_nsec = msg->header.stamp.nanosec;
    double time = static_cast<double>(t_sec) + 1e-9 * static_cast<double>(t_nsec);


    std::ofstream ofs(csv_file_path_, std::ios_base::app);
    if (!ofs) {
      std::filesystem::create_directories(std::filesystem::path(csv_file_path_).parent_path()); 
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", csv_file_path_.c_str());
      return;
    }
    ofs << time <<','<< output_msg.width * output_msg.height << "," << decompression_time.count() << "," << msg->data.size() <<"\n";
    ofs.close();
        


  }
  std::string csv_file_path_;
  rclcpp::Subscription<draco_ros2::msg::CompressedPointCloudDraco>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DracoDecompressorNode>());
  rclcpp::shutdown();
  return 0;
}