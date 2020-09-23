// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// static const std::string OPENCV_WINDOW = "Image window";

// class ImageConverter
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
//   image_transport::Subscriber image_sub_;
//   image_transport::Publisher image_pub_;

// public:
//   ImageConverter()
//     : it_(nh_)
//   {
//     // Subscrive to input video feed and publish output video feed
//     image_sub_ = it_.subscribe("/camera/image_raw", 1,
//       &ImageConverter::imageCb, this);
//     image_pub_ = it_.advertise("/image_converter/output_video", 1);

//     cv::namedWindow(OPENCV_WINDOW);
//   }

//   ~ImageConverter()
//   {
//     cv::destroyWindow(OPENCV_WINDOW);
//   }

//   void imageCb(const sensor_msgs::ImageConstPtr& msg)
//   {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }

//     // Draw an example circle on the video stream
//     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

//     // Update GUI Window
//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);

//     // Output modified video stream
//     image_pub_.publish(cv_ptr->toImageMsg());
//   }
// };

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "image_converter");
//   ImageConverter ic;
//   ros::spin();
//   return 0;
// }


#include "image_converter.h"



std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;

    auto pipe = popen(cmd, "r"); // get rid of shared_ptr

    if (!pipe) throw std::runtime_error("popen() failed!");

    while (!feof(pipe)) {
        if (fgets(buffer.data(), 128, pipe) != nullptr)
            result += buffer.data();
    }

    auto rc = pclose(pipe);

    if (rc == EXIT_SUCCESS) { // == 0

    } else if (rc == EXIT_FAILURE) {  // EXIT_FAILURE is not used by all programs, maybe needs some adaptation.

    }
    return result;
}

  ImageConverter::ImageConverter()
    : it_(nh_)
  {

    this->nh_.setParam("/classification_result", "None");

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
  }

  void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    try
    {
      this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


  }


bool ImageConverter::AWSCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  const char *labels[2] = {"red_box", "blue_box"};
  bool is_write_success = cv::imwrite("/home/mohamed/Desktop/ooo.jpg", this->cv_ptr->image);

  // exec("python /home/mohamed/catkin_ws/src/pick_and_sort/src/test_cv.py")  
  std::string result = exec("/home/mohamed/drive/boto3/venv/bin/python /home/mohamed/catkin_ws/src/pick_and_sort/src/test_cv.py");

  result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());

  ROS_INFO("INFERENCE RESULT: %s", result.c_str());
  ROS_INFO("AWSCALLBACKSERVICE RESULT: %s", result.c_str());

  
  
  this->nh_.setParam("/classification_result", result);


  return true;
}


// std::vector<std::byte> cvtImageToByte(cv::Mat img)
// // {
// //   // std::vector<std::byte> v_char;
// //   // for(int i = 0; i < img.rows; i++)
// //   //     for(int j = 0; j < img.cols; j++)
// //   //         v_char.push_back(*(uchar*)(img.data+ i*img.step + j));     
  


// }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}