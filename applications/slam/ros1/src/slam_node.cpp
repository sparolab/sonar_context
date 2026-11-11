// ROS 1 node: sonar images + an odometry pose stream (DISO's /direct_sonar/pose by default) -> keyframes,
// Sonar Context loops, iSAM2. Publishes the optimized path, the loops and the sonar map; writes a session on exit.
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>

#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "sonar_slam.hpp"

using namespace sonar_context;

class SlamNode {
 public:
  explicit SlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : it_(nh) {
    SlamOptions o;
    pnh.param("keyframe_distance", o.keyframe_distance, o.keyframe_distance);
    pnh.param("keyframe_yaw_deg", o.keyframe_yaw_deg, o.keyframe_yaw_deg);
    pnh.param("col_threshold", o.loop.col_threshold, 0.2);
    pnh.param("row_threshold", o.loop.row_threshold, 0.0);
    pnh.param("exclude_recent", o.loop.exclude_recent, 30);
    pnh.param("num_candidates", o.loop.num_candidates, 1);
    pnh.param("patch_range", o.loop.patch_range, 25);
    pnh.param("patch_beam", o.loop.patch_beam, 4);
    pnh.param("skip_range", o.loop.skip_range, 75);
    pnh.param("fov_deg", o.geometry.fov_deg, 130.0);
    pnh.param("max_range", o.geometry.max_range, 50.0);
    pnh.param("min_range", o.min_range, o.min_range);
    pnh.param("keep_percentile", o.keep_percentile, o.keep_percentile);
    pnh.param("min_peak", o.min_peak, o.min_peak);
    pnh.param("odom_sigma_xy", o.odom_sigma_xy, o.odom_sigma_xy);
    pnh.param("loop_sigma_xy", o.loop_sigma_xy, o.loop_sigma_xy);
    pnh.param("range_bins", range_bins_, 750);
    pnh.param("beams", beams_, 300);
    pnh.param<std::string>("image_layout", layout_, "fan");  // fan | range_beam | beam_range
    pnh.param<std::string>("save_dir", save_dir_, "");
    pnh.param("flip_frame", flip_, false);
    pnh.param<std::string>("frame_id", frame_id_, "map");
    std::string image_topic, pose_topic, transport;
    pnh.param<std::string>("image_topic", image_topic, "/son");
    pnh.param<std::string>("image_transport", transport, "compressed");
    pnh.param<std::string>("pose_topic", pose_topic, "/direct_sonar/pose");
    pnh.param<std::string>("pose_type", pose_type_, "pose");  // pose (PoseStamped) | odometry (nav_msgs/Odometry)
    slam_ = std::make_unique<SonarSlam>(o);

    image_sub_ = it_.subscribe(image_topic, 20, &SlamNode::on_image, this, image_transport::TransportHints(transport));
    if (pose_type_ == "odometry")
      pose_sub_ = nh.subscribe(pose_topic, 100, &SlamNode::on_odometry, this);
    else
      pose_sub_ = nh.subscribe(pose_topic, 100, &SlamNode::on_pose, this);
    path_pub_ = nh.advertise<nav_msgs::Path>("/sonar_context/path", 1, true);
    odom_path_pub_ = nh.advertise<nav_msgs::Path>("/sonar_context/odom_path", 1, true);
    loop_pub_ = nh.advertise<visualization_msgs::Marker>("/sonar_context/loops", 1, true);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sonar_context/map", 1, true);
    ROS_INFO("sonar_context_slam: images %s (%s, %s), poses %s", image_topic.c_str(), transport.c_str(),
             layout_.c_str(), pose_topic.c_str());
  }

  ~SlamNode() {
    if (!save_dir_.empty() && !slam_->keyframes().empty()) {
      slam_->save(save_dir_);
      std::printf("sonar_context_slam: %zu keyframes, %zu loops -> %s\n", slam_->keyframes().size(),
                  slam_->loops().size(), save_dir_.c_str());
    }
  }

 private:
  void on_image(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat g = cv_bridge::toCvShare(msg, "mono8")->image;
    if (flip_) cv::flip(g, g, layout_ == "beam_range" ? 0 : 1);  // mirror the beams
    std::lock_guard<std::mutex> lk(mu_);
    image_ = g.clone();
    image_time_ = msg->header.stamp.toSec();
  }
  void on_pose(const geometry_msgs::PoseStampedConstPtr& msg) { step(msg->pose, msg->header.stamp); }
  void on_odometry(const nav_msgs::OdometryConstPtr& msg) { step(msg->pose.pose, msg->header.stamp); }

  Image polar_of(const cv::Mat& m) const {
    cv::Mat f;
    m.convertTo(f, CV_32F);
    Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> e(
        f.ptr<float>(), f.rows, f.cols);
    if (layout_ == "fan") return fan_to_polar(Image(e), slam_->options().geometry.fov_deg, range_bins_, beams_);
    if (layout_ == "beam_range") return Image(e.transpose());
    return Image(e);
  }

  void step(const geometry_msgs::Pose& p, const ros::Time& stamp) {
    cv::Mat img;
    double t;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (image_.empty()) return;
      img = image_;
      t = image_time_;
    }
    // flip_frame: odometry in a z-down frame (DISO): turn it, and the image, into the z-up frame
    const double yaw = tf2::getYaw(p.orientation);
    const gtsam::Pose2 odom = flip_ ? gtsam::Pose2(p.position.x, -p.position.y, -yaw)
                                    : gtsam::Pose2(p.position.x, p.position.y, yaw);
    if (!slam_->add(t, odom, polar_of(img))) return;
    const auto& kfs = slam_->keyframes();
    if (auto lc = slam_->last_loop())
      ROS_INFO("loop %d -> %d (descriptor %.3f, correlation peak %.3f)", lc->query, lc->match,
               lc->descriptor_distance, lc->peak);
    publish(stamp, slam_->last_loop().has_value() || kfs.size() % 10 == 0);
  }

  void publish(const ros::Time& stamp, bool map) {
    const auto P = slam_->poses();
    const auto& kfs = slam_->keyframes();
    nav_msgs::Path path, odom;
    path.header.stamp = odom.header.stamp = stamp;
    path.header.frame_id = odom.header.frame_id = frame_id_;
    auto pose_msg = [&](const gtsam::Pose2& q) {
      geometry_msgs::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = q.x(), ps.pose.position.y = q.y();
      ps.pose.orientation.z = std::sin(q.theta() / 2), ps.pose.orientation.w = std::cos(q.theta() / 2);
      return ps;
    };
    for (size_t i = 0; i < P.size(); ++i) path.poses.push_back(pose_msg(P[i])), odom.poses.push_back(pose_msg(kfs[i].odom));
    path_pub_.publish(path);
    odom_path_pub_.publish(odom);

    visualization_msgs::Marker lm;
    lm.header = path.header;
    lm.ns = "loops", lm.type = visualization_msgs::Marker::LINE_LIST, lm.scale.x = 0.3;
    lm.color.r = 0.13, lm.color.g = 0.77, lm.color.b = 0.37, lm.color.a = 1.0;
    lm.pose.orientation.w = 1;
    for (const auto& l : slam_->loops())
      for (int k : {l.query, l.match}) {
        geometry_msgs::Point pt;
        pt.x = P[k].x(), pt.y = P[k].y();
        lm.points.push_back(pt);
      }
    loop_pub_.publish(lm);

    if (!map) return;
    sensor_msgs::PointCloud2 cloud;
    cloud.header = path.header;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    size_t n = 0;
    for (const auto& k : kfs) n += k.points.size();
    mod.resize(n);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (size_t i = 0; i < kfs.size(); ++i) {
      const double c = std::cos(P[i].theta()), s = std::sin(P[i].theta());
      for (const auto& p : kfs[i].points) {
        *x = c * p.x() - s * p.y() + P[i].x(), *y = s * p.x() + c * p.y() + P[i].y(), *z = 0;
        ++x, ++y, ++z;
      }
    }
    map_pub_.publish(cloud);
  }

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher path_pub_, odom_path_pub_, loop_pub_, map_pub_;
  std::unique_ptr<SonarSlam> slam_;
  std::mutex mu_;
  cv::Mat image_;
  double image_time_ = 0;
  int range_bins_ = 750, beams_ = 300;
  std::string layout_, save_dir_, frame_id_, pose_type_;
  bool flip_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sonar_context_slam");
  ros::NodeHandle nh, pnh("~");
  SlamNode node(nh, pnh);
  ros::spin();
  return 0;
}
