#include <stdio.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>

class UsbCamNode
{
public:
  ros::NodeHandle &node_;
  sensor_msgs::Image img_;

  std::string video_device_name_;
  std::string io_method_name_;
  int image_width_,image_height_;
  std::string pixel_format_name_;

  ros::Time next_time_;
  int count_;

  IplImage* src;
  CvCapture* capture;

  image_transport::Publisher image_pub;
  


  UsbCamNode(ros::NodeHandle &node) : node_(node)
  {
    src=NULL;
    capture=NULL;
    
    image_transport::ImageTransport it(node);

    image_pub = it.advertise("image_raw", 1);
    int camera_num = -1;
    ros::NodeHandle nh("~");
    nh.getParam("camera_index", camera_num);
    printf("%d", camera_num);
//cvCaptureFromCAM(-1) uses any camera
    if(NULL==(capture = cvCaptureFromCAM(camera_num)))
  	{
    		printf("\nError on cvCaptureFromCAM");
  	}
    if(NULL==(src=cvQueryFrame(capture))){
      printf("\nError on cvQueryFrame");
    }
   printf("Acquired image (%d/%d)\n",src->width,src->height);
   printf("Channels (%d)\n",src->nChannels);
   printf("Depth (%d)\n",src->depth);
  //-----
#ifdef OUTPUT_ENABLED
  cvNamedWindow("Capture", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Capture", 50, 50);
#endif
    next_time_ = ros::Time::now();
    count_ = 0;
  }

  virtual ~UsbCamNode()
  {
  	cvReleaseCapture(&capture);
#ifdef OUTPUT_ENABLED
  	cvDestroyWindow("Capture");
#endif
  }

  bool take_and_send_image()
  {
    int key;
    if(NULL==(src=cvQueryFrame(capture))){
      printf("\nError on cvQueryFrame");
      return false;
    }else{
#ifdef OUTPUT_ENABLED
    cvShowImage("Capture", src);
    key = cvWaitKey(10);
#endif
    // fill image message
    fillImage(img_, "bgr8", src->height,src->width, src->nChannels  * src->width, src->imageData);
    // publish
    image_pub.publish(img_);
    return true;
    }
  }


  bool spin()
  {
    while (node_.ok())
    {
      if (take_and_send_image())
      {
        count_++;
        ros::Time now_time = ros::Time::now();
        if (now_time > next_time_) {
          std::cout << count_ << " frames/sec at " << now_time << std::endl;
          count_ = 0;
          next_time_ = next_time_ + ros::Duration(1,0);
        }
      } else {
        ROS_ERROR("couldn't take image.");
        usleep(1000000);
      }
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv,"raspi_cam_ros");
  ros::NodeHandle n;
  UsbCamNode a(n);
  a.spin();
  return 0;
}
