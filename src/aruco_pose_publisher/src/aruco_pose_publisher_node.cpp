#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

struct Intrinsics {
  cv::Mat K;          // 3x3
  cv::Mat D;          // Nx1
  bool fisheye{false};
  bool valid() const { return !K.empty() && !D.empty(); }
};

static bool load_intrinsics(const std::string &path, Intrinsics &out) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Warning: cannot open intrinsics at " << path << "; pose estimation disabled.\n";
    return false;
  }
  auto read_first = [&](const std::vector<std::string> &keys) -> cv::Mat {
    for (const auto &k : keys) {
      cv::FileNode node = fs[k];
      if (!node.empty()) {
        cv::Mat m; node >> m; if (!m.empty()) return m;
      }
    }
    return cv::Mat();
  };

  out.K = read_first({"camera_matrix", "K", "M"});
  out.D = read_first({"distortion_coefficients", "distCoeffs", "D"});
  std::string model;
  if (!fs["distortion_model"].empty()) fs["distortion_model"] >> model;
  fs.release();

  out.fisheye = !model.empty() && (std::string("fisheye") == std::string(model.begin(), model.end()));

  if (!out.K.empty()) out.K.convertTo(out.K, CV_64F);
  if (!out.D.empty()) {
    out.D.convertTo(out.D, CV_64F);
    out.D = out.D.reshape(1, out.D.total()); // make it a column vector
  }
  if (!out.valid()) {
    std::cerr << "Warning: intrinsics missing keys in " << path << "; pose estimation disabled.\n";
    return false;
  }
  std::cout << "Intrinsics loaded (" << (out.fisheye?"fisheye":"pinhole") << ")";
  std::cout << ": K shape " << out.K.rows << "x" << out.K.cols << ", D len " << out.D.total() << "\n";
  return true;
}

static geometry_msgs::msg::Pose rvecTvecToPose(const cv::Mat &rvec, const cv::Mat &tvec) {
  // Rodrigues to rotation matrix
  cv::Mat R;
  cv::Rodrigues(rvec, R); // 3x3
  // Convert to quaternion (x,y,z,w)
  double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
  double qw, qx, qy, qz;
  if (trace > 0.0) {
    double s = std::sqrt(trace + 1.0) * 2.0; // s=4*qw
    qw = 0.25 * s;
    qx = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
    qy = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
    qz = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
  } else if ((R.at<double>(0,0) > R.at<double>(1,1)) && (R.at<double>(0,0) > R.at<double>(2,2))) {
    double s = std::sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2)) * 2.0; // s=4*qx
    qw = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
    qx = 0.25 * s;
    qy = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
    qz = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
  } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
    double s = std::sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2)) * 2.0; // s=4*qy
    qw = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
    qx = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
    qy = 0.25 * s;
    qz = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
  } else {
    double s = std::sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1)) * 2.0; // s=4*qz
    qw = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
    qx = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
    qy = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
    qz = 0.25 * s;
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = tvec.at<double>(0);
  pose.position.y = tvec.at<double>(1);
  pose.position.z = tvec.at<double>(2);
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  return pose;
}

class ArucoPoseNode : public rclcpp::Node {
public:
  ArucoPoseNode() : Node("aruco_pose_publisher") {
    // Parameters
    video_url_ = this->declare_parameter<std::string>("video_url", "http://10.152.70.62/video_raw/tracking_down");
    marker_size_ = this->declare_parameter<double>("marker_size", 0.112); // meters
    intrinsics_path_ = this->declare_parameter<std::string>("intrinsics_path", "");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_frame");

    poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco/poses", 10);
    ids_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("aruco/ids", 10);
    first_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("aruco/first_pose", 10);

    // Load intrinsics if provided
    if (!intrinsics_path_.empty()) {
      load_intrinsics(intrinsics_path_, intr_);
    } else {
      RCLCPP_WARN(this->get_logger(), "No intrinsics_path set. Will detect markers but skip pose unless provided.");
    }

    // Set up detector
    dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
    params_ = cv::aruco::DetectorParameters::create();

    // Open capture
    cap_.open(video_url_);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video source: %s", video_url_.c_str());
    }

    timer_ = this->create_wall_timer(33ms, std::bind(&ArucoPoseNode::onTimer, this)); // ~30Hz
  }

private:
  void onTimer() {
    if (!cap_.isOpened()) return;

    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) return;

    cv::Mat gray;
    if (frame.channels() == 3) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); else gray = frame;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(gray, dict_, corners, ids, params_, rejected);

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->now();
    pose_array.header.frame_id = frame_id_;

    std_msgs::msg::Int32MultiArray ids_msg;

    if (!ids.empty() && intr_.valid()) {
      for (size_t i = 0; i < ids.size(); ++i) {
        cv::Mat img_pts(corners[i]); // 4x1x2 implicit
        cv::Mat obj_pts(4, 3, CV_64F);
        double s = marker_size_ / 2.0;
        obj_pts.at<double>(0,0) = -s; obj_pts.at<double>(0,1) =  s; obj_pts.at<double>(0,2) = 0.0;
        obj_pts.at<double>(1,0) =  s; obj_pts.at<double>(1,1) =  s; obj_pts.at<double>(1,2) = 0.0;
        obj_pts.at<double>(2,0) =  s; obj_pts.at<double>(2,1) = -s; obj_pts.at<double>(2,2) = 0.0;
        obj_pts.at<double>(3,0) = -s; obj_pts.at<double>(3,1) = -s; obj_pts.at<double>(3,2) = 0.0;

        cv::Mat rvec, tvec;
        bool ok = false;

        if (intr_.fisheye) {
          // Undistort to ideal pixels using P=K
          cv::Mat pts( (int)corners[i].size(), 1, CV_64FC2);
          for (int j=0; j<pts.rows; ++j) {
            pts.at<cv::Vec2d>(j,0)[0] = static_cast<double>(corners[i][j].x);
            pts.at<cv::Vec2d>(j,0)[1] = static_cast<double>(corners[i][j].y);
          }
          cv::Mat undist;
          cv::fisheye::undistortPoints(pts, undist, intr_.K, intr_.D, cv::noArray(), intr_.K);
          std::vector<cv::Point2d> undist_pts;
          undist_pts.reserve(4);
          for (int j=0; j<undist.rows; ++j) {
            auto v = undist.at<cv::Vec2d>(j,0);
            undist_pts.emplace_back(v[0], v[1]);
          }
          ok = cv::solvePnP(obj_pts, undist_pts, intr_.K, cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        } else {
          std::vector<cv::Point2d> pts;
          pts.reserve(4);
          for (const auto &p : corners[i]) pts.emplace_back(p.x, p.y);
          ok = cv::solvePnP(obj_pts, pts, intr_.K, intr_.D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        }

        if (ok) {
          geometry_msgs::msg::Pose pose = rvecTvecToPose(rvec, tvec);
          pose_array.poses.push_back(pose);
          ids_msg.data.push_back(ids[i]);
          // Publish first pose too
          if (pose_array.poses.size() == 1) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = pose_array.header;
            ps.pose = pose;
            first_pose_pub_->publish(ps);
          }
        }
      }
    }

    poses_pub_->publish(pose_array);
    ids_pub_->publish(ids_msg);
  }

  // Params
  std::string video_url_;
  double marker_size_;
  std::string intrinsics_path_;
  std::string frame_id_;

  // OpenCV
  cv::VideoCapture cap_;
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;

  // Intrinsics
  Intrinsics intr_;

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ids_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr first_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
