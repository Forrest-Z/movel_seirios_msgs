/*
Indoor image segmentation
Date: 02/07/2020
Author: Thuong

We use python for image segmentation, so I stop coding in this file C++.
We can start it again if necessary in the future
To run this file, we need to install torch for C++
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <movel_hasp_vendor/license.h>

// #include <torch/torch.h>
// #include <torch/script.h>
// #include <iostream>

#include <time.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // PSP model constant
  const float ADE20K_MEAN[3] = {104.00699, 116.66877, 122.67892};
  // Size of model
  const int IMG_H = 473;
  const int IMG_W = 473;

  // Load PSPNET model
  // Deserialize the ScriptModule from a file using torch::jit::load().
  // torch::jit::script::Module module = torch::jit::load("pspnet_50_ade20k.pth");

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  // Callback for image from sensor msg
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Start time
    clock_t tStart = clock();

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // predict(cv_ptr->image);

    // End running time
    std::cout << clock() - tStart << " ms";
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  /*
  Do semantic segmentation
  Args: image
  */
  void predict(cv::Mat img_ros)
  {
    cv::Mat img = img_ros.clone();

    // Resize the image
    cv::resize(img, img, cv::Size(IMG_H, IMG_W), 0, 0, CV_INTER_LINEAR);

    // Convert to float32
    img.convertTo(img, CV_32FC3);

    // Subtract the image by mean of dataset
    cv::subtract(img, Scalar(ADE20K_MEAN[0], ADE20K_MEAN[1], ADE20K_MEAN[2]), img);
    /*
    // The channel dimension is the last dimension in OpenCV
    at::Tensor tensor_image = torch::from_blob(img.data, {1, img.rows, img.cols, 3}, at::kByte);
    tensor_image = tensor_image.to(at::kFloat);

    // Transpose the image for [channels, rows, columns] format of pytorch tensor
    tensor_image = at::transpose(tensor_image, 1, 2);
    tensor_image = at::transpose(tensor_image, 1, 3);

    // Create a vector of torch inputs
    std::vector<torch::jit::IValue> input;
    input.emplace_back(tensor_image);

    // Execute the model and turn its output into a tensor.
    // auto output = module.forward(input).toTensor().clone().squeeze(0);
    */
  }
};

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE                                                                                                    
    MovelLicense ml(17);                                                                                                   
    if (!ml.login())                                                                                                      
      return 1;                                                                                                           
  #endif
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  #ifdef MOVEL_LICENSE                                                                                                    
    ml.logout();          
  #endif    

  return 0;
}
