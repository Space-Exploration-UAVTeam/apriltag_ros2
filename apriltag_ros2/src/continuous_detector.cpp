#include "apriltag_ros2/common_functions.h"
#include <memory>
using std::placeholders::_1;
class ContinuousDetector : public rclcpp::Node 
// class ContinuousDetector : public std::enable_shared_from_this<rclcpp::Node> 
{
 public:
  ContinuousDetector() : Node("ContinuousDetector")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "ContinuousDetector!!!");

    // tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(shared_from_this()));
    // tag_detector_ = std::make_shared<TagDetector>(shared_from_this());//terminate called after throwing an instance of 'std::bad_weak_ptr'

    this->declare_parameter("publish_tag_detections_image", true);
    draw_tag_detections_image_ = this->get_parameter("publish_tag_detections_image").as_bool();//https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Parameter.html


		rclcpp::QoS qos(rclcpp::KeepLast(3),rmw_qos_profile_sensor_data);
    camera_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/hikrobot/image_left", qos, 
                                    std::bind(&ContinuousDetector::imageCallback, this, _1));
    tag_detections_publisher_ = this->create_publisher<apriltag_ros2_interfaces::msg::AprilTagDetectionArray>("tag_detections", 10);
    if (draw_tag_detections_image_)
    {
      tag_detections_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("tag_detections_image", 10);
    }

    // timer = this->create_wall_timer(500ms, std::bind(&ContinuousDetector::initialization, this));
  }
  ~ContinuousDetector() = default;


 private:
  std::shared_ptr<TagDetector> tag_detector_;
  bool draw_tag_detections_image_;
  cv_bridge::CvImagePtr cv_image_;
  bool init_detector_ = false;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscriber_;
  rclcpp::Publisher<apriltag_ros2_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tag_detections_image_publisher_;

  // void initialization()
  // {
  //   tag_detector_ = std::make_shared<TagDetector>(shared_from_this());
  // }

  void imageCallback (const sensor_msgs::msg::Image::SharedPtr image_rect)
  {
    if(!init_detector_)
    {
      tag_detector_ = std::make_shared<TagDetector>(shared_from_this());
      init_detector_ = true;
    }
    // Lazy updates: When there are no subscribers _and_ when tf is not published, skip detection.
    if (tag_detections_publisher_->get_subscription_count() == 0 && tag_detections_image_publisher_->get_subscription_count() == 0)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "No subscribers, skip processing.");
      return;
    }
    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run AprilTag 2 on the iamge
    try
    {
      cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Publish detected tags in the image by AprilTag 
    apriltag_ros2_interfaces::msg::AprilTagDetectionArray tag_detection_array(tag_detector_->detectTags(cv_image_));
    tag_detections_publisher_->publish(tag_detection_array);

    // Publish the camera image overlaid by outlines of the detected tags and their payload values
    if (draw_tag_detections_image_)
    {
      tag_detector_->drawDetections(cv_image_);

      tag_detections_image_publisher_->publish(*(cv_image_->toImageMsg()));
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContinuousDetector>());
  rclcpp::shutdown();
  return 0;
}