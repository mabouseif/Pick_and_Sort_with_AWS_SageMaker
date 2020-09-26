#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_srvs/Empty.h>

static const std::string OPENCV_WINDOW = "Image window";


// Class for converting from ROS images to OpenCV images
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr;
    ros::ServiceServer my_service = nh_.advertiseService("/AWS_service", &ImageConverter::AWSCallback, this);

    bool AWSCallback(std_srvs::EmptyRequest  &req, std_srvs::EmptyResponse &res);
    

  public:
    ImageConverter();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    // std::vector<std::byte> cvtImageToByte(cv::Mat img);

};