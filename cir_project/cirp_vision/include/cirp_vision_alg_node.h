// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _cirp_vision_alg_node_h_
#define _cirp_vision_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "cirp_vision_alg.h"

// [publisher subscriber headers]
#include <cirp_vision/CirpDetectedPieces.h>
//#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
// Uncomment to use the openCV <-> ROS bridge
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_listener.h>

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class CirpVisionAlgNode : public algorithm_base::IriBaseAlgorithm<CirpVisionAlgorithm>
{
  private:
    tf::TransformListener listener_;

    // [publisher attributes]
    ros::Publisher pieces_publisher_;

    //camera_info_manager::CameraInfoManager feedback_camera_manager;
    //image_transport::CameraPublisher feedback_publisher_;
    //sensor_msgs::Image feedback_Image_msg_;
    ros::Publisher feedback_publisher_;


    // [subscriber attributes]
    ros::Subscriber points_subscriber_;
    void points_callback(const PointCloud::ConstPtr& msg);
    pthread_mutex_t points_mutex_;
    void points_mutex_enter(void);
    void points_mutex_exit(void);

    ros::Subscriber img_in_subscriber_;
    //image_transport::CameraSubscriber img_in_subscriber_;
    void img_in_callback(const sensor_msgs::Image::ConstPtr& msg);
    //void img_in_callback(const sensor_msgs::Image::ConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& info);
    pthread_mutex_t img_in_mutex_;
    void img_in_mutex_enter(void);
    void img_in_mutex_exit(void);

    // Uncomment to use the openCV <-> ROS bridge
    cv_bridge::CvImageConstPtr cv_image_; // last image
    //image_transport::ImageTransport it;

    PointCloud::ConstPtr cloud_; // last point cloud

    void transform_point(const std_msgs::Header& header,
        geometry_msgs::Point& point);

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    CirpVisionAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~CirpVisionAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
