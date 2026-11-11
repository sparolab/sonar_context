// ROS 2 node: sonar images + an odometry pose stream (DISO's /direct_sonar/pose by default, e.g. recorded with
// ROS 1) -> keyframes, Sonar Context loops, iSAM2. Same back-end and parameters as the ROS 1 node.
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "sonar_slam.hpp"

using namespace sonar_context;
using std::placeholders::_1;

static double yaw_of(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

class SlamNode : public rclcpp::Node {
 public:
  SlamNode() : Node("sonar_context_slam") {
    SlamOptions o;
    o.keyframe_distance = declare_parameter("keyframe_distance", o.keyframe_distance);
    o.keyframe_yaw_deg = declare_parameter("keyframe_yaw_deg", o.keyframe_yaw_deg);
    o.loop.col_threshold = declare_parameter("col_threshold", 0.2);
    o.loop.row_threshold = declare_parameter("row_threshold", 0.0);
    o.loop.exclude_recent = declare_parameter("exclude_recent", 30);
    o.loop.num_candidates = declare_parameter("num_candidates", 1);
    o.loop.patch_range = declare_parameter("patch_range", 25);
    o.loop.patch_beam = declare_parameter("patch_beam", 4);
    o.loop.skip_range = declare_parameter("skip_range", 75);
    o.geometry.fov_deg = declare_parameter("fov_deg", 130.0);
    o.geometry.max_range = declare_parameter("max_range", 50.0);
    o.min_range = declare_parameter("min_range", o.min_range);
    o.keep_percentile = declare_parameter("keep_percentile", o.keep_percentile);
    o.min_peak = declare_parameter("min_peak", o.min_peak);
    o.odom_sigma_xy = declare_parameter("odom_sigma_xy", o.odom_sigma_xy);
    o.loop_sigma_xy = declare_parameter("loop_sigma_xy", o.loop_sigma_xy);
    range_bins_ = declare_parameter("range_bins", 750);
    beams_ = declare_parameter("beams", 300);
    layout_ = declare_parameter<std::string>("image_layout", "fan");
    save_dir_ = declare_parameter<std::string>("save_dir", "");
    flip_ = declare_parameter("flip_frame", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const auto image_topic = declare_parameter<std::string>("image_topic", "/son");
    const auto transport = declare_parameter<std::string>("image_transport", "compressed");
    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/direct_sonar/pose");
    const auto pose_type = declare_parameter<std::string>("pose_type", "pose");
    slam_ = std::make_unique<SonarSlam>(o);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(20));
    if (transport == "compressed")
      csub_ = create_subscription<sensor_msgs::msg::CompressedImage>(image_topic + "/compressed", qos,
          [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
            set_image(cv::imdecode(m->data, cv::IMREAD_GRAYSCALE), rclcpp::Time(m->header.stamp).seconds());
          });
    else
      isub_ = create_subscription<sensor_msgs::msg::Image>(image_topic, qos,
          [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
            set_image(cv_bridge::toCvShare(m, "mono8")->image.clone(), rclcpp::Time(m->header.stamp).seconds());
          });
    if (pose_type == "odometry")
      osub_ = create_subscription<nav_msgs::msg::Odometry>(pose_topic, 100,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { step(m->pose.pose, m->header.stamp); });
    else
      psub_ = create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 100,
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { step(m->pose, m->header.stamp); });
    const auto latched = rclcpp::QoS(1).transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/sonar_context/path", latched);
    odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("/sonar_context/odom_path", latched);
    loop_pub_ = create_publisher<visualization_msgs::msg::Marker>("/sonar_context/loops", latched);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/sonar_context/map", latched);
    RCLCPP_INFO(get_logger(), "images %s (%s, %s), poses %s", image_topic.c_str(), transport.c_str(), layout_.c_str(),
                pose_topic.c_str());
  }

  ~SlamNode() override {
    if (!save_dir_.empty() && !slam_->keyframes().empty()) {
      slam_->save(save_dir_);
      std::printf("sonar_context_slam: %zu keyframes, %zu loops -> %s\n", slam_->keyframes().size(),
                  slam_->loops().size(), save_dir_.c_str());
    }
  }

 private:
  void set_image(cv::Mat g, double t) {
    if (g.empty()) return;
    if (flip_) cv::flip(g, g, layout_ == "beam_range" ? 0 : 1);  // mirror the beams
    std::lock_guard<std::mutex> lk(mu_);
    image_ = g, image_time_ = t;
  }

  Image polar_of(const cv::Mat& m) const {
    cv::Mat f;
    m.convertTo(f, CV_32F);
    Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> e(f.ptr<float>(), f.rows, f.cols);
    if (layout_ == "fan") return fan_to_polar(Image(e), slam_->options().geometry.fov_deg, range_bins_, beams_);
    if (layout_ == "beam_range") return Image(e.transpose());
    return Image(e);
  }

  void step(const geometry_msgs::msg::Pose& p, const builtin_interfaces::msg::Time& stamp) {
    cv::Mat img;
    double t;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (image_.empty()) return;
      img = image_, t = image_time_;
    }
    // flip_frame: odometry in a z-down frame (DISO): turn it, and the image, into the z-up frame
    const double yaw = yaw_of(p.orientation);
    const gtsam::Pose2 odom = flip_ ? gtsam::Pose2(p.position.x, -p.position.y, -yaw)
                                    : gtsam::Pose2(p.position.x, p.position.y, yaw);
    if (!slam_->add(t, odom, polar_of(img))) return;
    if (auto lc = slam_->last_loop())
      RCLCPP_INFO(get_logger(), "loop %d -> %d (descriptor %.3f, correlation peak %.3f)", lc->query,
                  lc->match, lc->descriptor_distance, lc->peak);
    publish(stamp, slam_->last_loop().has_value() || slam_->keyframes().size() % 10 == 0);
  }

  void publish(const builtin_interfaces::msg::Time& stamp, bool map) {
    const auto P = slam_->poses();
    const auto& kfs = slam_->keyframes();
    nav_msgs::msg::Path path, odom;
    path.header.stamp = odom.header.stamp = stamp;
    path.header.frame_id = odom.header.frame_id = frame_id_;
    auto pose_msg = [&](const gtsam::Pose2& q) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = q.x(), ps.pose.position.y = q.y();
      ps.pose.orientation.z = std::sin(q.theta() / 2), ps.pose.orientation.w = std::cos(q.theta() / 2);
      return ps;
    };
    for (size_t i = 0; i < P.size(); ++i) path.poses.push_back(pose_msg(P[i])), odom.poses.push_back(pose_msg(kfs[i].odom));
    path_pub_->publish(path);
    odom_path_pub_->publish(odom);

    visualization_msgs::msg::Marker lm;
    lm.header = path.header;
    lm.ns = "loops", lm.type = visualization_msgs::msg::Marker::LINE_LIST, lm.scale.x = 0.3;
    lm.color.r = 0.13, lm.color.g = 0.77, lm.color.b = 0.37, lm.color.a = 1.0;
    lm.pose.orientation.w = 1;
    for (const auto& l : slam_->loops())
      for (int k : {l.query, l.match}) {
        geometry_msgs::msg::Point pt;
        pt.x = P[k].x(), pt.y = P[k].y();
        lm.points.push_back(pt);
      }
    loop_pub_->publish(lm);

    if (!map) return;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = path.header;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    size_t n = 0;
    for (const auto& k : kfs) n += k.points.size();
    mod.resize(n);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (size_t i = 0; i < kfs.size(); ++i) {
      const double c = std::cos(P[i].theta()), s = std::sin(P[i].theta());
      for (const auto& q : kfs[i].points) {
        *x = c * q.x() - s * q.y() + P[i].x(), *y = s * q.x() + c * q.y() + P[i].y(), *z = 0;
        ++x, ++y, ++z;
      }
    }
    map_pub_->publish(cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr csub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr isub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr psub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr osub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, odom_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  std::unique_ptr<SonarSlam> slam_;
  std::mutex mu_;
  cv::Mat image_;
  double image_time_ = 0;
  int range_bins_ = 750, beams_ = 300;
  std::string layout_, save_dir_, frame_id_;
  bool flip_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
