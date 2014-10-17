#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cv_bridge/cv_bridge.h>

#include <kinect2_definitions.h>

#include <depth_registration.h>

class CloudPublisher
{
public:
  /**
   * Default Constructor
   */
  CloudPublisher();

  /*
   * Destructor
   */
  ~CloudPublisher();

  /**
   * Initialize the communication with ros
   */
  void initialize();

private:
  void onImagesReceived(const sensor_msgs::Image::ConstPtr p_color_image, const sensor_msgs::Image::ConstPtr p_depth_image, const sensor_msgs::CameraInfo::ConstPtr p_color_camera_info, const sensor_msgs::CameraInfo::ConstPtr p_depth_camera_info);
  void updateCameraMatrix(const sensor_msgs::CameraInfo::ConstPtr p_camera_info, cv::Mat& camera_matrix);
  void updateCvImage(const sensor_msgs::Image::ConstPtr p_ros_image, cv::Mat& cv_image);

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  ros::AsyncSpinner spinner_;
  ros::Publisher point_cloud_publisher_;
  image_transport::SubscriberFilter* p_color_image_subscriber_filter_;
  image_transport::SubscriberFilter* p_depth_image_subscriber_filter_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> * p_color_camera_info_subscriber_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> * p_depth_camera_info_subscriber_;
  message_filters::Synchronizer<ExactSyncPolicy> * p_exact_synchronizer_;
  message_filters::Synchronizer<ApproximateSyncPolicy> * p_approximate_synchronizer_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_point_cloud_;
  
  DepthRegistration* p_depth_registration_;

  bool use_exact_synchronization_;

  cv::Mat x_lookup_;
  cv::Mat y_lookup_;
};

CloudPublisher::CloudPublisher()
  : node_handle_("~")
  , image_transport_(node_handle_)
  , spinner_(0)
  , p_depth_registration_(NULL)
  , use_exact_synchronization_(true)
{
}

CloudPublisher::~CloudPublisher()
{
  if (use_exact_synchronization_)
  {
    delete p_exact_synchronizer_;
  }
  else
  {
    delete p_approximate_synchronizer_;
  }

  delete p_color_image_subscriber_filter_;
  delete p_depth_image_subscriber_filter_;
  delete p_color_camera_info_subscriber_;
  delete p_depth_camera_info_subscriber_;

  if (p_depth_registration_)
  {
    delete p_depth_registration_;
  }
}

void CloudPublisher::initialize()
{
  std::string color_image_topic, depth_image_topic, color_camera_info_topic, depth_camera_info_topic, point_cloud_topic;
  int queue_size;
  
  node_handle_.param("color_image_topic", color_image_topic, std::string("/" K2_TOPIC_RECT_COLOR K2_TOPIC_RAW));
  node_handle_.param("depth_image_topic", depth_image_topic, std::string("/" K2_TOPIC_REG_DEPTH K2_TOPIC_RAW));
  node_handle_.param("ros_queue_size", queue_size, 5);

  color_camera_info_topic = color_image_topic.substr(0, color_image_topic.rfind('/')) + std::string("/camera_info");
  depth_camera_info_topic = depth_image_topic.substr(0, depth_image_topic.rfind('/')) + std::string("/camera_info");

  point_cloud_topic = color_image_topic.substr(0, color_image_topic.rfind('/'));
  point_cloud_topic = point_cloud_topic.substr(0, point_cloud_topic.rfind('/')) + std::string("/points");

  point_cloud_publisher_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(point_cloud_topic, 1);

  p_color_image_subscriber_filter_ = new image_transport::SubscriberFilter(image_transport_, color_image_topic, queue_size, image_transport::TransportHints(std::string("compressed")));
  p_depth_image_subscriber_filter_ = new image_transport::SubscriberFilter(image_transport_, depth_image_topic, queue_size, image_transport::TransportHints(std::string("compressedDepth")));

  p_color_camera_info_subscriber_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(node_handle_, color_camera_info_topic, queue_size);
  p_depth_camera_info_subscriber_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(node_handle_, depth_camera_info_topic, queue_size);

  if (use_exact_synchronization_)
  {
    p_exact_synchronizer_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queue_size), *p_color_image_subscriber_filter_, *p_depth_image_subscriber_filter_, *p_color_camera_info_subscriber_, *p_depth_camera_info_subscriber_);
    p_exact_synchronizer_->registerCallback(boost::bind(&CloudPublisher::onImagesReceived, this, _1, _2, _3, _4));
  }
  else
  {
    p_approximate_synchronizer_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queue_size), *p_color_image_subscriber_filter_, *p_depth_image_subscriber_filter_, *p_color_camera_info_subscriber_, *p_depth_camera_info_subscriber_);
    p_approximate_synchronizer_->registerCallback(boost::bind(&CloudPublisher::onImagesReceived, this, _1, _2, _3, _4));
  }

  spinner_.start();

  ros::waitForShutdown();

  spinner_.stop();
}

void CloudPublisher::onImagesReceived(const sensor_msgs::Image::ConstPtr p_color_image, const sensor_msgs::Image::ConstPtr p_depth_image, const sensor_msgs::CameraInfo::ConstPtr p_color_camera_info, const sensor_msgs::CameraInfo::ConstPtr p_depth_camera_info)
{
  cv::Mat color_camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat depth_camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
  
  updateCameraMatrix(p_color_camera_info, color_camera_matrix);
  updateCameraMatrix(p_depth_camera_info, depth_camera_matrix);

  cv::Mat color_image, depth_image, scaled_depth_image;

  updateCvImage(p_color_image, color_image);
  updateCvImage(p_depth_image, depth_image);

  if ( (depth_image.rows != color_image.rows) ||
       (depth_image.cols != color_image.cols) )
  {
    if (p_depth_registration_ == NULL)
    {
      //p_depth_registration_ = DepthRegistration::New(cv::Size(color_image.cols, color_image.rows), cv::Size(depth_image.cols, depth_image.rows), cv::Size(depth_image.cols, depth_image.rows), 0.5f, 20.0f, 0.015f, DepthRegistration::CPU);
      p_depth_registration_ = DepthRegistration::New(cv::Size(color_image.cols, color_image.rows), cv::Size(depth_image.cols, depth_image.rows), cv::Size(depth_image.cols, depth_image.rows), 0.5f, 20.0f, 0.015f, DepthRegistration::OPENCL);
      p_depth_registration_->init(color_camera_matrix, depth_camera_matrix, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 3, CV_64F), cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_32F), cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_32F));
    }

    p_depth_registration_->depthToRGBResolution(depth_image, scaled_depth_image);
  }
  else
  {
    scaled_depth_image = depth_image;
  }

  if (p_point_cloud_ == NULL)
  {
    p_point_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    p_point_cloud_->height = color_image.rows;
    p_point_cloud_->width = color_image.cols;
    p_point_cloud_->is_dense = false;
    p_point_cloud_->points.resize(p_point_cloud_->height * p_point_cloud_->width);
    p_point_cloud_->header.frame_id = p_color_image->header.frame_id;

    const float fx = 1.0f / color_camera_matrix.at<double>(0, 0);
    const float fy = 1.0f / color_camera_matrix.at<double>(1, 1);
    const float cx = color_camera_matrix.at<double>(0, 2);
    const float cy = color_camera_matrix.at<double>(1, 2);

    y_lookup_ = cv::Mat(1, color_image.rows, CV_32F);
    float* p_element = y_lookup_.ptr<float>();
    
    for (int r = 0; r < color_image.rows; ++r, ++p_element)
    {
      *p_element = (r - cy) * fy;
    }

    x_lookup_ = cv::Mat(1, color_image.cols, CV_32F);
    p_element = x_lookup_.ptr<float>();
    
    for (int c = 0; c < color_image.cols; ++c, ++p_element)
    {
      *p_element = (c - cx) * fx;
    }
  } // if (p_point_cloud_ == NULL)

  const float bad_point = std::numeric_limits<float>::quiet_NaN();

  #pragma omp parallel for
  for(int r = 0; r < scaled_depth_image.rows; ++r)
  {
    pcl::PointXYZRGB* p_cloud_element = &p_point_cloud_->points[r * scaled_depth_image.cols];
    const uint16_t* p_depth_element = scaled_depth_image.ptr<uint16_t>(r);
    const cv::Vec3b* p_color_element = color_image.ptr<cv::Vec3b>(r);

    const float y = y_lookup_.at<float>(0, r);
    const float* p_x_element = x_lookup_.ptr<float>();

    for (size_t c = 0; c < (size_t)scaled_depth_image.cols; ++c, ++p_cloud_element, ++p_depth_element, ++p_color_element, ++p_x_element)
    {
      register const float depth_value = *p_depth_element / 1000.0f;

      // check for invalid measurements
      if (isnan(depth_value) || depth_value <= 0.001)
      {
        // not valid
        p_cloud_element->x = bad_point;
        p_cloud_element->y = bad_point;
        p_cloud_element->z = bad_point;
        p_cloud_element->rgb = 0;

        continue;
      }
      
      p_cloud_element->z = depth_value;
      p_cloud_element->x = *p_x_element * depth_value;
      p_cloud_element->y = y * depth_value;
      p_cloud_element->b = p_color_element->val[0];
      p_cloud_element->g = p_color_element->val[1];
      p_cloud_element->r = p_color_element->val[2];
    }
  }

  p_point_cloud_->header.stamp = p_color_image->header.stamp.toNSec();

  point_cloud_publisher_.publish(p_point_cloud_);
}

void CloudPublisher::updateCameraMatrix(const sensor_msgs::CameraInfo::ConstPtr p_camera_info, cv::Mat& camera_matrix)
{
  double *p_element = camera_matrix.ptr<double>(0, 0);
  
  for (size_t i = 0; i < 9; ++i, ++p_element)
  {
    *p_element = p_camera_info->K[i];
  }
}

void CloudPublisher::updateCvImage(const sensor_msgs::Image::ConstPtr p_ros_image, cv::Mat& cv_image)
{
  cv_bridge::CvImageConstPtr p_cv_image;
  p_cv_image = cv_bridge::toCvShare(p_ros_image, p_ros_image->encoding);
  p_cv_image->image.copyTo(cv_image);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_publisher");

  try
  {
    CloudPublisher cloud_publisher;
    cloud_publisher.initialize();
  }
  catch (std::exception& exception)
  {
    std::cout << exception.what();
  }

  ros::shutdown();

  return 0;
}
