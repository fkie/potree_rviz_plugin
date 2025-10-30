#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

using namespace std::chrono_literals;

class PointCloudTestPublisher : public rclcpp::Node
{
public:
    PointCloudTestPublisher() : Node("pointcloud_test_publisher")
    {
        width_ = declare_parameter<int>("grid_width", 640);
        height_ = declare_parameter<int>("grid_height", 480);
        spacing_ = declare_parameter<double>("grid_spacing", 0.01);
        z_distance_ = declare_parameter<double>("grid_distance", 5.0);

        if (width_ <= 0)
            width_ = 640;
        if (height_ <= 0)
            height_ = 480;
        if (spacing_ <= 0.0)
            spacing_ = 0.01;

        generateGridCloud();

        publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud>("test_pointcloud", rclcpp::QoS(1).transient_local());
        timer_ = create_wall_timer(100ms, std::bind(&PointCloudTestPublisher::publishPointCloud, this));
    }

private:
    void generateGridCloud()
    {
        base_cloud_.channels.clear();
        base_cloud_.points.clear();
        base_cloud_.channels.resize(1);
        sensor_msgs::msg::ChannelFloat32& rgb_channel = base_cloud_.channels.front();
        rgb_channel.name = "rgb";

        base_cloud_.points.reserve(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_));
        rgb_channel.values.reserve(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_));

        double origin_x = -0.5 * (static_cast<double>(width_) - 1) * spacing_;
        double origin_y = -0.5 * (static_cast<double>(height_) - 1) * spacing_;
        double z = z_distance_;

        for (int v = 0; v < height_; ++v)
        {
            double y = origin_y + static_cast<double>(v) * spacing_;
            for (int u = 0; u < width_; ++u)
            {
                double x = origin_x + static_cast<double>(u) * spacing_;
                geometry_msgs::msg::Point32 p;
                p.x = static_cast<float>(x);
                p.y = static_cast<float>(y);
                p.z = static_cast<float>(z);
                base_cloud_.points.push_back(p);

                float u_norm = static_cast<float>(u) / static_cast<float>(std::max(1, width_ - 1));
                float v_norm = static_cast<float>(v) / static_cast<float>(std::max(1, height_ - 1));
                std::uint8_t r = static_cast<std::uint8_t>(std::round(u_norm * 255.0f));
                std::uint8_t g = static_cast<std::uint8_t>(std::round(v_norm * 255.0f));
                std::uint8_t b = static_cast<std::uint8_t>(std::round((1.0f - u_norm) * 255.0f));
                std::uint8_t a = 255;
                std::uint32_t rgba = (static_cast<std::uint32_t>(a) << 24) |
                                     (static_cast<std::uint32_t>(r) << 16) |
                                     (static_cast<std::uint32_t>(g) << 8) |
                                     static_cast<std::uint32_t>(b);
                float packed;
                std::memcpy(&packed, &rgba, sizeof(float));
                rgb_channel.values.push_back(packed);
            }
        }
    }

    void publishPointCloud()
    {
        sensor_msgs::msg::PointCloud msg = base_cloud_;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int width_;
    int height_;
    double spacing_;
    double z_distance_;
    sensor_msgs::msg::PointCloud base_cloud_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTestPublisher>());
    rclcpp::shutdown();
    return 0;
}
