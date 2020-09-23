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
  bool is_write_success = cv::imwrite("/home/mohamed/Desktop/object_image.jpg", this->cv_ptr->image);

  // exec("python /home/mohamed/catkin_ws/src/pick_and_sort/src/aws_sagemaker_client.py")  
  std::string result = exec("/home/mohamed/drive/boto3/venv/bin/python /home/mohamed/catkin_ws/src/pick_and_sort/src/aws_sagemaker_client.py");

  result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());

  ROS_INFO("INFERENCE RESULT: %s", result.c_str());
  ROS_INFO("AWSCALLBACKSERVICE RESULT: %s", result.c_str());

  
  
  this->nh_.setParam("/classification_result", result);


  return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}