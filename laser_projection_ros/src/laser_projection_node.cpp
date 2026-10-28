#include "laser_projection_core/backends.hpp"
#include "laser_projection_core/projector.hpp"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

class LaserProjectionNode final : public rclcpp::Node {
public:
  LaserProjectionNode() : Node("laser_projection_node"), tf_buffer_(get_clock()),
                          tf_listener_(tf_buffer_) {
    path_topic_ = declare_parameter("path_topic", std::string("/plan"));
    projection_frame_ = declare_parameter("projection_frame", std::string("map"));
    projector_frame_ = declare_parameter("projector_frame", std::string("laser_projector"));
    color_r_ = declare_parameter("color_r", 0);
    color_g_ = declare_parameter("color_g", 255);
    color_b_ = declare_parameter("color_b", 0);
    color_topic_ = declare_parameter("color_topic", std::string("/laser_color"));
    point_spacing_ = declare_parameter("point_spacing", 0.05);
    arrows_ = declare_parameter("arrows", false);
    demo_arrow_length_ = declare_parameter("demo_arrow_length", 0.5);
    scan_rate_ = declare_parameter("scan_rate", 20000.0);
    surface_z_ = declare_parameter("surface_z", 0.0);
    auto backend_name = declare_parameter("backend", std::string("udp"));
    std::shared_ptr<laser_projection::LaserBackend> backend;
    if (backend_name == "helios")
      backend = std::make_shared<laser_projection::HeliosBackend>(
          declare_parameter("helios_library", std::string("libHeliosDacAPI.so")));
    else
      backend = std::make_shared<laser_projection::UdpBackend>(
          declare_parameter("udp_host", std::string("127.0.0.1")),
          declare_parameter("udp_port", 2345));
    projector_ = std::make_unique<laser_projection::Projector>(
        std::make_shared<laser_projection::PlaneSurface>(surface_z_), backend);
    if (!projector_->initialize()) {
      RCLCPP_ERROR(get_logger(), "Laser backend failed to initialize");
    } else {
      RCLCPP_INFO(get_logger(), "Laser backend initialized (%s)", backend_name.c_str());
    }
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, rclcpp::QoS(10),
        std::bind(&LaserProjectionNode::path_callback, this, std::placeholders::_1));
    color_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
        color_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::ColorRGBA::SharedPtr message) {
          color_r_ = static_cast<int>(std::clamp(message->r, 0.0f, 1.0f) * 255.0f);
          color_g_ = static_cast<int>(std::clamp(message->g, 0.0f, 1.0f) * 255.0f);
          color_b_ = static_cast<int>(std::clamp(message->b, 0.0f, 1.0f) * 255.0f);
          if (demo_active_)
            draw_demo_arrows(demo_arrow_count_);
          else if (last_path_)
            path_callback(last_path_);
        });
    command_sub_ = create_subscription<std_msgs::msg::String>(
        "/laser_command", rclcpp::QoS(10),
        std::bind(&LaserProjectionNode::command_callback, this, std::placeholders::_1));
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/debug_path", 10);
  }

private:
  // ponytail: accept a tiny literal command grammar until Nav2 paths drive shapes.
  void command_callback(const std_msgs::msg::String::SharedPtr message) {
    std::istringstream command(message->data);
    std::string action;
    command >> action;
    if (action == "clear") {
      std::string extra;
      if (command >> extra) {
        RCLCPP_WARN(get_logger(), "Usage: clear");
        return;
      }
      demo_active_ = false;
      demo_arrow_count_ = 0;
      last_path_.reset();
      projector_->show_paths({}, {{}, {}});
      return;
    }
    if (action != "arrow") {
      RCLCPP_WARN(get_logger(), "Unknown command; use 'arrow [count]' or 'clear'");
      return;
    }
    int count = 1;
    if (command >> count) {
      std::string extra;
      if (command >> extra) {
        RCLCPP_WARN(get_logger(), "Usage: arrow [count], where count is 1-20");
        return;
      }
    } else if (!command.eof()) {
      RCLCPP_WARN(get_logger(), "Usage: arrow [count], where count is 1-20");
      return;
    }
    if (count < 1 || count > 20) {
      RCLCPP_WARN(get_logger(), "Arrow count must be between 1 and 20");
      return;
    }
    demo_active_ = true;
    demo_arrow_count_ = count;
    draw_demo_arrows(count);
  }

  void draw_demo_arrows(int count) {
    if (!std::isfinite(demo_arrow_length_) || demo_arrow_length_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "demo_arrow_length must be finite and positive");
      return;
    }
    constexpr double gap = 0.2;
    const double total_width = count * demo_arrow_length_ + (count - 1) * gap;
    std::vector<std::vector<laser_projection::Point3D>> paths;
    paths.reserve(count);
    for (int i = 0; i < count; ++i) {
      const double x = -total_width / 2.0 + i * (demo_arrow_length_ + gap);
      paths.push_back({{x, 0.0, surface_z_},
                       {x + demo_arrow_length_, 0.0, surface_z_}});
    }
    try {
      const auto projector_tf = tf_buffer_.lookupTransform(
          projection_frame_, projector_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.05));
      const auto& t = projector_tf.transform.translation;
      const auto& q = projector_tf.transform.rotation;
      laser_projection::PathOptions options;
      options.color = {static_cast<unsigned char>(color_r_),
                       static_cast<unsigned char>(color_g_),
                       static_cast<unsigned char>(color_b_)};
      options.point_spacing = point_spacing_;
      options.arrows = true;
      projector_->show_paths(paths, {{t.x, t.y, t.z}, {q.x, q.y, q.z, q.w}},
                             options, scan_rate_);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Could not transform demo arrows: %s", error.what());
    }
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr message) {
    try {
      if (message->header.frame_id.empty()) {
        RCLCPP_WARN(get_logger(), "Ignoring path with empty header.frame_id");
        return;
      }
      last_path_ = message;
      demo_active_ = false;
      const auto stamp = message->header.stamp;
      auto projector_tf = tf_buffer_.lookupTransform(
          projection_frame_, projector_frame_, stamp, tf2::durationFromSec(0.05));
      std::vector<laser_projection::Point3D> path;
      path.reserve(message->poses.size());
      for (const auto& pose : message->poses) {
        auto pose_with_frame = pose;
        if (pose_with_frame.header.frame_id.empty())
          pose_with_frame.header.frame_id = message->header.frame_id;
        geometry_msgs::msg::PoseStamped transformed;
        tf_buffer_.transform(pose_with_frame, transformed, projection_frame_,
                             tf2::durationFromSec(0.05));
        path.push_back({transformed.pose.position.x, transformed.pose.position.y,
                        transformed.pose.position.z});
      }
      const auto& t = projector_tf.transform.translation;
      const auto& q = projector_tf.transform.rotation;
      laser_projection::PathOptions options;
      options.color = {static_cast<unsigned char>(color_r_),
                       static_cast<unsigned char>(color_g_),
                       static_cast<unsigned char>(color_b_)};
      options.point_spacing = point_spacing_;
      options.arrows = arrows_;
      projector_->show_path(path, {{t.x, t.y, t.z}, {q.x, q.y, q.z, q.w}},
                            options, scan_rate_);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = projection_frame_;
      marker.header.stamp = now();
      marker.ns = "laser_projection";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.03;
      marker.color.g = 1.0;
      marker.color.a = 1.0;
      for (const auto& point : path) {
        geometry_msgs::msg::Point p;
        p.x = point.x; p.y = point.y; p.z = point.z;
        marker.points.push_back(p);
      }
      marker_pub_->publish(marker);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Could not transform path: %s", error.what());
    }
  }

  std::string path_topic_;
  std::string projection_frame_;
  std::string projector_frame_;
  std::string color_topic_;
  int color_r_{};
  int color_g_{};
  int color_b_{};
  double scan_rate_{20000.0};
  double point_spacing_{0.05};
  double demo_arrow_length_{0.5};
  double surface_z_{};
  bool arrows_{false};
  bool demo_active_{false};
  int demo_arrow_count_{};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<laser_projection::Projector> projector_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  nav_msgs::msg::Path::SharedPtr last_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
