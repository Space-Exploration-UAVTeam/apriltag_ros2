#ifndef APRILTAG_ROS_COMMON_FUNCTIONS_H
#define APRILTAG_ROS_COMMON_FUNCTIONS_H

#include <string>
#include <sstream>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"		
// #include <XmlRpcException.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <apriltag.h>
#include "apriltag_ros2_interfaces/msg/april_tag_detection.hpp"
#include "apriltag_ros2_interfaces/msg/april_tag_detection_array.hpp"

#include "common/homography.h"
#include "tagStandard52h13.h"
#include "tagStandard41h12.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCustom48h12.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
////////////////////////////////////////// camera parameters and variables/////////////////////////////////////////////
const cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 397.30,    0.0, 320.13,
                                                    0.0, 394.03, 262.50,
                                                    0.0,    0.0,    1.0 );//
// [k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,taux,tauy] of 4, 5, 8, 12 or 14 elements.
// If the vector is empty, the zero distortion coefficients are assumed.
const cv::Mat D = ( cv::Mat_<double> ( 4,1 ) << -3.5897074026113568e-01, 9.6480686476447072e-02, -2.0615787184859718e-03, -6.1006160991551920e-04);
const int ImgWidth = 1280;
const int ImgHeight = 1024;
static cv::Mat map1_;
static cv::Mat map2_;

// template<typename T>
// T getAprilTagOption(ros::NodeHandle& pnh, const std::string& param_name, const T & default_val)
// {
//   T param_val;
//   pnh.param<T>(param_name, param_val, default_val);
//   return param_val;
// }

// Stores the properties of a tag member of a bundle
struct TagBundleMember
{
  int id; // Payload ID
  double size; // [m] Side length
  cv::Matx44d T_oi; // Rigid transform from tag i frame to bundle origin frame
};

class StandaloneTagDescription
{
 public:
  StandaloneTagDescription() {};
  StandaloneTagDescription(int id, double size,
                           std::string &frame_name) :
      id_(id),
      size_(size),
      frame_name_(frame_name) {}

  double size() { return size_; }
  int id() { return id_; }
  std::string& frame_name() { return frame_name_; }

 private:
  // Tag description
  int id_;
  double size_;
  std::string frame_name_;
};

class TagBundleDescription
{
 public:
  std::map<int, int > id2idx_; // (id2idx_[<tag ID>]=<index in tags_>) mapping

  TagBundleDescription(std::string name) :
      name_(name) {}

  void addMemberTag(int id, double size, cv::Matx44d T_oi) {
    TagBundleMember member;
    member.id = id;
    member.size = size;
    member.T_oi = T_oi;
    tags_.push_back(member);
    id2idx_[id] = tags_.size()-1;
  }

  std::string name () const { return name_; }
  // Get IDs of bundle member tags
  std::vector<int> bundleIds () {
    std::vector<int> ids;
    for (unsigned int i = 0; i < tags_.size(); i++) {
      ids.push_back(tags_[i].id);
    }
    return ids;
  }
  // Get sizes of bundle member tags
  std::vector<double> bundleSizes () {
    std::vector<double> sizes;
    for (unsigned int i = 0; i < tags_.size(); i++) {
      sizes.push_back(tags_[i].size);
    }
    return sizes;
  }
  int memberID (int tagID) { return tags_[id2idx_[tagID]].id; }
  double memberSize (int tagID) { return tags_[id2idx_[tagID]].size; }
  cv::Matx44d memberT_oi (int tagID) { return tags_[id2idx_[tagID]].T_oi; }

 private:
  // Bundle description
  std::string name_;
  std::vector<TagBundleMember > tags_;
};

class TagDetector
{
 private:
  // Detections sorting
  static int idComparison(const void* first, const void* second);

  // Remove detections of tags with the same ID
  void removeDuplicates();

  /////////////////////////////////////////AprilTag 2 code's attributes///////////////////////////////////////////////
  std::string family_ = "tag36h11";
  int threads_ = 2;
  double decimate_ = 1.0;
  double blur_ = 0.0;
  int refine_edges_ = 1;
  int debug_= 0;
  int max_hamming_distance_ = 2;  // Tunable, but really, 2 is a good choice. Values of >=3
                                  // consume prohibitively large amounts of memory, and otherwise you want the largest value possible.

  ///////////// remove all XmlRpcValue for parameters reading from .yaml files,for ROS2 do not support!!!!////////////
  std::map<int, StandaloneTagDescription> standalone_tag_descriptions_;
  bool remove_duplicates_ = true;


  // AprilTag 2 objects
  apriltag_family_t *tf_;
  apriltag_detector_t *td_;
  zarray_t *detections_;

  // Other members
  std::vector<TagBundleDescription > tag_bundle_descriptions_;
  bool run_quietly_;
  // bool publish_tf_;
  // tf::TransformBroadcaster tf_pub_;

  // rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Node> node_;

 public:
  TagDetector(rclcpp::Node::SharedPtr node); 
  // TagDetector(std::shared_ptr<rclcpp::Node> node);
  // TagDetector(ros::NodeHandle pnh);
  ~TagDetector();

  // Store standalone and bundle tag descriptions
  // std::map<int, StandaloneTagDescription> parseStandaloneTags(XmlRpc::XmlRpcValue& standalone_tag_descriptions);
  // std::vector<TagBundleDescription > parseTagBundles(XmlRpc::XmlRpcValue& tag_bundles);
  // double xmlRpcGetDouble(XmlRpc::XmlRpcValue& xmlValue, std::string field) const;
  // double xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue& xmlValue, std::string field, double defaultValue) const;

  bool findStandaloneTagDescription(
      int id, StandaloneTagDescription*& descriptionContainer,
      bool printWarning = true);

  geometry_msgs::msg::PoseWithCovarianceStamped makeTagPose(
      const Eigen::Matrix4d& transform,
      const Eigen::Quaternion<double> rot_quaternion,
      const std_msgs::msg::Header& header);

  // Detect tags in an image
  apriltag_ros2_interfaces::msg::AprilTagDetectionArray detectTags(const cv_bridge::CvImagePtr& image);

  // Get the pose of the tag in the camera frame
  // Returns homogeneous transformation matrix [R,t;[0 0 0 1]] which
  // takes a point expressed in the tag frame to the same point
  // expressed in the camera frame. As usual, R is the (passive)
  // rotation from the tag frame to the camera frame and t is the
  // vector from the camera frame origin to the tag frame origin,
  // expressed in the camera frame.

  Eigen::Matrix4d getRelativeTransform(std::vector<cv::Point3d > objectPoints, std::vector<cv::Point2d > imagePoints) const;

  void addImagePoints(apriltag_detection_t *detection,
                      std::vector<cv::Point2d >& imagePoints) const;
  void addObjectPoints(double s, cv::Matx44d T_oi,
                       std::vector<cv::Point3d >& objectPoints) const;

  // Draw the detected tags' outlines and payload values on the image
  void drawDetections(cv_bridge::CvImagePtr image);

  // bool get_publish_tf() const { return publish_tf_; }
};

#endif // APRILTAG_ROS_COMMON_FUNCTIONS_H
