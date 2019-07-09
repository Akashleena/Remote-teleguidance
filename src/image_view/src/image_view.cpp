

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <image_view/ImageViewConfig.h>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
 #include "image_geometry/pinhole_camera_model.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/subscriber_filter.h>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <opencv/cv.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>


#include "geometry_msgs/TransformStamped.h"
#include<opencv2/core/core.hpp>

#include <sensor_msgs/distortion_models.h>
#include <boost/make_shared.hpp>

using namespace std;
using namespace tf;

using namespace cv;


#include <nodelet/loader.h>
//float distval;
int g_count;
//cv::Mat g_last_image;
cv_bridge::CvImageConstPtr last_cv_ptr;
boost::format g_filename_format;
boost::mutex g_image_mutex;
std::string g_window_name;
bool g_gui;
ros::Publisher g_pub;
bool g_do_dynamic_scaling;
int g_colormap;
double g_min_image_value;
double g_max_image_value;



image_geometry::PinholeCameraModel cam_model_;	
cv_bridge::CvImageConstPtr depthimagePtr;



void reconfigureCb(image_view::ImageViewConfig &config, uint32_t level)
{
		
	  boost::mutex::scoped_lock lock(g_image_mutex);
	  g_do_dynamic_scaling = config.do_dynamic_scaling;
	  g_colormap = config.colormap;
	  g_min_image_value = config.min_image_value;
	  g_max_image_value = config.max_image_value;
	
	
}

                                                                /*camera info, depth info input*/
void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::ImageConstPtr& imageDepth,const sensor_msgs::CameraInfoConstPtr& camInfo,const sensor_msgs::CameraInfoConstPtr& camDepthInfo)
                          
{
//	 static tf::TransformBroadcaster br;
       /* tf::TransformListener tf_listener_;
	tf::StampedTransform transform;*/

// 	tf::Transform transform;
  	cv_bridge::CvImageConstPtr cv_ptr;

	//ROS_INFO("Image: HI!");
		{
  			boost::mutex::scoped_lock lock(g_image_mutex);
			cam_model_.fromCameraInfo(camDepthInfo);

  	// Convert to OpenCV native BGR color

  			try {
    				cv_bridge::CvtColorForDisplayOptions options;
    				options.do_dynamic_scaling = g_do_dynamic_scaling;
    				options.colormap = g_colormap;
    	// Set min/max value for scaling to visualize depth/float image.


    				if (g_min_image_value == g_max_image_value)
				{


      	// Not specified by rosparam, then set default value.
      // Because of current sensor limitation, we use 10m as default of max range of depth
      // with consistency to the configuration in rqt_image_view.


      					options.min_image_value = 0;
      					if (msg->encoding == "32FC1") 
					{
        				options.max_image_value = 10;  // 10 [m]
 					ROS_INFO ("32FC1 ENCODING");
      					} 

	
					else if (msg->encoding == "16UC1") 
					{
        				options.max_image_value = 10 * 1000;  // 10 * 1000 [mm]
					ROS_INFO ("16UC1 ENCODING");

     	 				}

					else if (msg->encoding == "8UC1") 
					{
        				options.max_image_value =  1000;  // 1000 [mm]
					ROS_INFO ("8UC1 ENCODING");

     	 				}
					else if (msg->encoding == "8UC2") 
					{
        				options.max_image_value =  1000;  // 1000 [mm]
					ROS_INFO ("8UC2 ENCODING");

     	 				}
					else if (msg->encoding == "8UC3") 
					{
        				options.max_image_value =  1000;  // 1000 [mm]
					ROS_INFO ("8UC3 ENCODING");
					}
					else if (msg->encoding == "8UC4") 
					{
        				options.max_image_value =  1000;  // 1000 [mm]
					ROS_INFO ("8UC4 ENCODING");

     	 				}

     	 				
					else
					{
					ROS_INFO_STREAM(msg->encoding);

}

    				  }//end of if 


					else
	 				{
      					options.min_image_value = g_min_image_value;
      					options.max_image_value = g_max_image_value;
    					}

    

	cv_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(msg), "", options);

	depthimagePtr = cv_bridge::toCvShare(imageDepth);
 
    	last_cv_ptr=cv_ptr;

    	cv::Mat g_last_image = cv_ptr->image;

	int COLUMN_VAL= g_last_image.cols;

	int ROW_VAL=g_last_image.rows;

	
  				       }//end of try

 			catch (cv_bridge::Exception& e)
	 			{
   					ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'",msg->encoding.c_str(), e.what());
  	  			}
// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map","/camera_depth_optical_frame" ));

try{
          // tf_listener_.lookupTransform("/base_link","/camera_depth_optical_frame",ros::Time(0), transform);

		 //tf_listener_.lookupTransform("map","camera_depth_optical_frame",ros::Time::now(), transform);
		//ROS_INFO_STREAM(transform);
}
catch (tf::TransformException ex){
    ROS_WARN("Base to camera transform unavailable %s", ex.what());
    }


  }//end of imgcb
   //ROS_INFO("Image: BYE");		

  					if (g_gui && !cv_ptr->image.empty())
      					{
   						const cv::Mat &image = cv_ptr->image;
    						cv::imshow(g_window_name, image);
   						cv::waitKey(1);
  					}

  					if (g_pub.getNumSubscribers() > 0) 
					{
    						g_pub.publish(cv_ptr);
  					}

  }

 static void mouseCb(int event, int x, int y, int flags, void* param)
  {
	//ROS_INFO("MINE");
    // return;
  					if (event == cv::EVENT_LBUTTONDOWN) 
  					{
    						ROS_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
 	  					ROS_INFO("lEFT MOUSE IS CLICKED with x and y co-ordinates as:[%d,%d]",x,y);
    						return;
  					} 



					else if (event == cv::EVENT_RBUTTONDOWN)
 					{
   
					ROS_INFO("RIGHT MOUSE IS CLICKED with x and y co-ordinates as:[%d,%d]",x,y);

    					return;
  					}


					//ROS_INFO("Mouse: HI!");


cv_bridge::CvImageConstPtr depthPtr;
cv_bridge::CvImageConstPtr imagePtr;

{
	boost::mutex::scoped_lock lock(g_image_mutex);
	depthPtr=depthimagePtr;
	imagePtr=last_cv_ptr;
}

if(!depthPtr || !imagePtr) //if you click the mouse before getting the image
{

ROS_INFO("Click the mouse before getting the image.");
return;
}

  const cv::Mat& image = imagePtr->image; //temp_im;

  					if (image.empty())
 					{
    					ROS_WARN("Couldn't save image, no data!");
   					return;
  					}

  std::cout << image.rows << " " << image.channels() << " " << image.type() << std::endl;
// i uncommented these lines


  std::cout << image.at<cv::Vec3b>(y,x) << std::endl;
 

  //std::cout << image.at<float>(x,y) << std::endl;



const cv::Mat& dimage = depthPtr->image;

  float distval=dimage.at<float>(y,x);

  std::cout << "Depth: " << distval << std::endl;

 std::vector<cv::Mat> channels(3);
 // split img:
 cv::split(image, channels);
 int pixel = channels[0].at<uchar>(0,0);
 //std::cout << pixel << std::endl;
                                       
  /*display the rgb values*/
  //cv::Vec3b pixel = image.at<cv::Vec3b>(0,0);
  // ROS_INFO("image[%d,%d]=(r=%d,g=%d,b=%d)",y,x,pixel[2],pixel[1],pixel[0]);
  

  cv::Point2d uv_rect;

  uv_rect.x=x;
  uv_rect.y=y;

  cv::Point3d ray=cam_model_.projectPixelTo3dRay (uv_rect);
  ray=ray*distval;
  ROS_INFO_STREAM("Co-ordinates in camera frame"<<(ray));

}

static void guiCb(const ros::WallTimerEvent&)
{
    // Process pending GUI events and return immediately
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_view", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");
  ros::WallTimer gui_timer;
  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  local_nh.param("window_name", g_window_name, topic);
  local_nh.param("gui", g_gui, true);  // gui/no_gui mode
	 if (g_gui) 
			{

   			 std::string format_string;
   			 local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
   			 g_filename_format.parse(format_string);

   		 	// Handle window size
   		 	bool autosize;
  		 	local_nh.param("autosize", autosize, false);
   		 	cv::namedWindow(g_window_name, autosize ? (cv::WINDOW_AUTOSIZE |cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED) : 0);

   			 cv::setMouseCallback(g_window_name, &mouseCb);

   			 if(autosize == false)
   			 {
     			 if(local_nh.hasParam("width") && local_nh.hasParam("height"))
      				{
       					 int width;
        				local_nh.getParam("width", width);
        				int height;
       					 local_nh.getParam("height", height);
      				 	 cv::resizeWindow(g_window_name, width, height);
     				 }
    		          }

    
   		 	gui_timer = local_nh.createWallTimer(ros::WallDuration(0.1), guiCb);
  			}


  // Handle transport
  // priority:
  //    1. command line argument
  //    2. rosparam '~image_transport'
  std::string transport;
  local_nh.param("image_transport", transport, std::string("raw"));
  ros::V_string myargv;
  ros::removeROSArgs(argc, argv, myargv);

  			for (size_t i = 1; i < myargv.size(); ++i)
 			{
    			if (myargv[i][0] != '-')
   			{
      			transport = myargv[i];
     			 break;
    			}
  			}
  ROS_INFO_STREAM("Using transport \"" << transport << "\"");

 
  g_pub = local_nh.advertise<sensor_msgs::Image>("output", 1);

  image_transport::SubscriberFilter image_sub_;

  image_transport::SubscriberFilter image_depth_sub_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> info_depth_sub_;


   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,sensor_msgs::CameraInfo> MyApproxSyncPolicy;

   message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;

   typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncPolicy;

   message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

   ros::NodeHandle camera_nh(nh, "camera");

   ros::NodeHandle rgb_nh(camera_nh, "rgb");

   ros::NodeHandle depth_nh(camera_nh, "depth");

   ros::NodeHandle rgb_pnh(local_nh, "rgb");
   
   ros::NodeHandle depth_pnh(local_nh, "depth");
   
   image_transport::ImageTransport rgb_it(rgb_nh);

   image_transport::ImageTransport depth_it(depth_nh);

                                                /*transport rbg and depth images*/

   image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		
   image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

   int queueSize = 1000;

   bool approxSync = true;
		

   local_nh.param("queue_size", queueSize, queueSize);

   local_nh.param("approx_sync", approxSync, approxSync);

   ROS_INFO("Approximate time sync = %s", approxSync?"true":"false");


		if(approxSync)
		{
			approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize), image_sub_, image_depth_sub_, info_sub_, info_depth_sub_);
			approxSync_->registerCallback(boost::bind(&imageCb, _1, _2, _3, _4));
		}
		else
		{
			exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize), image_sub_, image_depth_sub_,  info_sub_, info_depth_sub_);
			exactSync_->registerCallback(boost::bind(&imageCb, _1, _2, _3, _4));
		}

		image_sub_.subscribe(rgb_it, rgb_nh.resolveName("image_raw"), 1, hintsRgb);
		image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image_raw"), 1, hintsDepth);
		info_sub_.subscribe(rgb_nh, "camera_info", 1);
		info_depth_sub_.subscribe(depth_nh, "camera_info", 1);

		
  dynamic_reconfigure::Server<image_view::ImageViewConfig> srv;
  dynamic_reconfigure::Server<image_view::ImageViewConfig>::CallbackType f =
  boost::bind(&reconfigureCb, _1, _2);
  srv.setCallback(f);

  //ros::MultiThreadedSpinner spinner(4);
  //spinner.spin();
  ros::spin();

  		if (g_gui)
 		{
    		cv::destroyWindow(g_window_name);
  		}
  // The publisher is a global variable, and therefore its scope exceeds those
  // of the node handles in main(). Unfortunately, this will cause a crash
  // when the publisher tries to shut down and all node handles are gone
  // already. Therefore, we shut down the publisher now and avoid the annoying
  // mutex assertion.

  g_pub.shutdown();
  return 0;

  }


