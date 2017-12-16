#include "cirp_vision_alg_node.h"

CirpVisionAlgNode::CirpVisionAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<CirpVisionAlgorithm>()
  //feedback_camera_manager(ros::NodeHandle("~feedback")),
  //it(this->public_node_handle_)
{
  //init class attributes if necessary
  this->loop_rate_ = 10;//in [Hz]

  // [init publishers]
  this->pieces_publisher_ = this->public_node_handle_.advertise<cirp_vision::CirpDetectedPieces>("pieces", 100);
  this->feedback_publisher_ =
    this->public_node_handle_.advertise<sensor_msgs::Image>(
        "feedback/image_raw", 10);
  //this->feedback_publisher_ = this->it.advertiseCamera("feedback/image_raw", 10);
  // uncomment the following lines to load the calibration file for the camera
  // Change <cal_file_param> for the correct parameter name holding the configuration filename
  //std::string feedback_cal_file;
  //public_node_handle_.param<std::string>("<cal_file_param>",feedback_cal_file,"");
  //if(this->feedback_camera_manager.validateURL(feedback_cal_file))
  //{
  //  if(!this->feedback_camera_manager.loadCameraInfo(feedback_cal_file))
  //    ROS_INFO("Invalid calibration file");
  //}
  //else
  //  ROS_INFO("Invalid calibration file");

  
  // [init subscribers]
  this->points_subscriber_ = this->public_node_handle_.subscribe("points", 10,
      &CirpVisionAlgNode::points_callback, this);
  pthread_mutex_init(&this->points_mutex_,NULL);

  this->img_in_subscriber_ = this->public_node_handle_.subscribe(
      "img_in/image_raw", 10, &CirpVisionAlgNode::img_in_callback, this);
  //this->img_in_subscriber_ = this->it.subscribeCamera("img_in/image_raw", 10, &CirpVisionAlgNode::img_in_callback, this);
  pthread_mutex_init(&this->img_in_mutex_,NULL);

  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

CirpVisionAlgNode::~CirpVisionAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->points_mutex_);
  pthread_mutex_destroy(&this->img_in_mutex_);
}

void CirpVisionAlgNode::transform_point(const std_msgs::Header& header,
    geometry_msgs::Point& point)
{
  geometry_msgs::PointStamped in, out;
  in.header = header;
  in.point = point;
  listener_.transformPoint(config_.frame_tgt, in, out);
  point = out.point;
}

void CirpVisionAlgNode::mainNodeThread(void)
{
  this->alg_.lock();

  if (cv_image_ and cloud_) // check first that the node has received an image
  {
    cirp_vision::CirpDetectedPieces detected;
    detected.header = cv_image_->header;
    cv_bridge::CvImage feedback;
    feedback.header = cv_image_->header;
    feedback.encoding = "bgr8";
    alg_.segmentate(cv_image_->image, feedback.image, *cloud_, detected);
    if (not config_.frame_tgt.empty())
    {
      listener_.waitForTransform(config_.frame_tgt, detected.header.frame_id,
          ros::Time(0), ros::Duration(30.0));
      for (int idx = 0; idx < detected.positions.size(); ++idx)
      {
        transform_point(detected.header, detected.positions[idx]);
      }
      detected.header.frame_id = config_.frame_tgt;
    }
    feedback_publisher_.publish(feedback.toImageMsg());
    pieces_publisher_.publish(detected);
  }

  this->alg_.unlock();

  // [fill msg structures]
  // Initialize the topic message structure
  //this->pieces_CirpDetectedPieces_msg_.data = my_var;

  // Initialize the topic message structure
  //this->feedback_Image_msg_.data = my_var;

  // Uncomment the following lines two initialize the camera info structure
  //sensor_msgs::CameraInfo feedback_camera_info=this->feedback_camera_manager.getCameraInfo();
  //feedback_camera_info.header.stamp = <time_stamp>;
  //feedback_camera_info.header.frame_id = <frame_id>;
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->pieces_publisher_.publish(this->pieces_CirpDetectedPieces_msg_);

  // Uncomment the following line to convert an OpenCV image to a ROS image message
  //this->feedback_Image_msg_=*this->cv_image_->toImageMsg();
  // Uncomment the following line to publish the image together with the camera information
  //this->feedback_publisher_.publish(this->feedback_Image_msg_,feedback_camera_info);

}

/*  [subscriber callbacks] */
void CirpVisionAlgNode::points_callback(const PointCloud::ConstPtr& msg)
{
  //ROS_INFO("CirpVisionAlgNode::points_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  //this->points_mutex_enter();

  this->cloud_ = msg;

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  this->alg_.unlock();
  //this->points_mutex_exit();
}

void CirpVisionAlgNode::points_mutex_enter(void)
{
  pthread_mutex_lock(&this->points_mutex_);
}

void CirpVisionAlgNode::points_mutex_exit(void)
{
  pthread_mutex_unlock(&this->points_mutex_);
}

void CirpVisionAlgNode::img_in_callback(const sensor_msgs::Image::ConstPtr& msg)
{
  //ROS_INFO("CirpVisionAlgNode::img_in_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  //this->img_in_mutex_enter();

  //std::cout << msg->data << std::endl;
  // Uncomment the following line to convert the input image to OpenCV format
  this->cv_image_ = cv_bridge::toCvShare(msg, "bgr8");

  //unlock previously blocked shared variables
  this->alg_.unlock();
  //this->img_in_mutex_exit();
}

void CirpVisionAlgNode::img_in_mutex_enter(void)
{
  pthread_mutex_lock(&this->img_in_mutex_);
}

void CirpVisionAlgNode::img_in_mutex_exit(void)
{
  pthread_mutex_unlock(&this->img_in_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void CirpVisionAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void CirpVisionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<CirpVisionAlgNode>(argc, argv, "cirp_vision_alg_node");
}
