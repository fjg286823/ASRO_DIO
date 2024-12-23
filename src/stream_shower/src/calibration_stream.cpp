#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
// #include <dvs_msgs/Event.h>
// #include <dvs_msgs/EventArray.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <pangolin/pangolin.h>
// #include <ctime>
#include <chrono>
#include <vector>
#include<iostream>
#include<fstream>
#include <sys/stat.h>
#include <sys/types.h>
// #include <sstream>

using namespace std;
using namespace std::chrono;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void chatterOneCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("chatter one heard: [%s]", msg->data.c_str());
// }

// void chatterTwoCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("chatter two heared [%s]", msg->data.c_str());
// }
const int kinect_color_width =  640;
const int kinect_color_height = 480;
// const int davis_color_width =  346;
// const int davis_color_height = 260;
pangolin::View kinect_color_cam;
pangolin::View kinect_depth_cam; 
// pangolin::View davis_color_cam;
// pangolin::View davis_event_cam;
cv::Mat kinect_color(kinect_color_height, kinect_color_width, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Mat kinect_depth(kinect_color_height, kinect_color_width, CV_16UC1, cv::Scalar(0, 0, 0));
// cv::Mat davis_color(davis_color_height, davis_color_width, CV_8UC3, cv::Scalar(0, 0, 0));
// cv::Mat davis_event(davis_color_height, davis_color_width, CV_8UC3, cv::Scalar(0, 0, 0));


// unsigned char* imageArray = new unsigned char[davis_color_width*davis_color_height];

struct Record_image{
  milliseconds timestamp;
  cv::Mat map;
};

struct Record_event{
  milliseconds timestamp;
  int x;
  int y;
  int p;
};

// struct Kinect_color_maps{
//   time_t timestamp;
//   cv::Mat color;
// }

// struct Kinect_depth_maps{
//   time_t timestamp;
//   cv::Mat depth;
// }
bool record_streaming=false;
// vector<Record_image> v_events_images;
vector<Record_image> v_color_images;
vector<Record_image> v_depth_images;
// vector<Record_event> v_events;



void KinectColorCallback(const sensor_msgs::Image& msg)
{
  // ROS_INFO("receive color image [%d %d]", msg.height,msg.width);
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  kinect_color=cv_ptr->image;
  if (record_streaming){
    Record_image color_temp;
    color_temp.timestamp=duration_cast< milliseconds >(
    system_clock::now().time_since_epoch());
    color_temp.map=kinect_color;
    v_color_images.push_back(color_temp);
  }

}

void KinectDepthCallback(const sensor_msgs::Image& msg)
{
  // ROS_INFO("receive depth map [%d %d]", msg.height,msg.width);
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // std::cout<<msg.encoding.c_str()<<std::endl;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  kinect_depth=cv_ptr->image;
  // cv_ptr->image.convertTo(kinect_depth, CV_16UC1,  1000);
  if (record_streaming){
    Record_image color_temp;
    color_temp.timestamp=duration_cast< milliseconds >(
    system_clock::now().time_since_epoch());
    color_temp.map=kinect_depth.clone();
    v_depth_images.push_back(color_temp);
  }
}

// void DavisColorCallback(const sensor_msgs::Image& msg)
// {
//   // ROS_INFO("receive depth map [%d %d]", msg.height,msg.width);
//   // ROS_INFO("receive color image [%d %d]", msg.height,msg.width);
//   cv_bridge::CvImagePtr cv_ptr;
//   try
//   {
//     // std::cout<<msg.encoding.c_str()<<std::endl;
//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//     return;
//   }
//   cv::cvtColor(cv_ptr->image, davis_color, CV_RGB2BGR);

//   // cv::cvtColor(cv_ptr->image, davis_color, CV_RGB2GRAY);

//   // cv::imshow("aaa",davis_color);
//   // cv::waitKey(3);
//   // davis_color=cv_ptr->image;
// }


// void DavisEventCallback(const dvs_msgs::EventArray msg)
// {
//   cv::Mat davis_event_map(davis_color_height, davis_color_width, CV_8UC1, 255);

//   for (int i = 0; i < msg.events.size(); ++i)
//   {
//     const int x = msg.events[i].x;
//     const int y = msg.events[i].y;
//     // davis_event_map.at<cv::Vec3b>(cv::Point(x, y)) = (
//     //     msg.events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
//     davis_event_map.at<uchar>(y, x) = 0;

//     Record_event a_event;
//     a_event.timestamp=duration_cast< milliseconds >(
//     system_clock::now().time_since_epoch());
//     a_event.x=x;
//     a_event.y=y;
//     if (msg.events[i].polarity){
//       a_event.p=1;
//     }else{
//       a_event.p=0;
//     }
//     v_events.push_back(a_event);

//   }
//   davis_event=davis_event_map;
//   if (record_streaming){
//     Record_image color_temp;
//     color_temp.timestamp=duration_cast< milliseconds >(
//     system_clock::now().time_since_epoch());
//     color_temp.map=davis_event;
//     v_events_images.push_back(color_temp);
//   }
//   // printf("height:%d width%d",davis_event.size().height,davis_event.size().width);



// }


// void setImageData(unsigned char * imageArray, int size){
//   for(int i = 0 ; i < size;i++) {
//     imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
//   }
// }

// void setImageData_withmat(unsigned char * imageArray, int size,cv::Mat mat){
//   for (int i=0;i< mat.size().height;i++){
//     for (int j=0;j<mat.size().width;j++){
//       imageArray[i*davis_color_width+j] = mat.at<uchar>(i,j);
//     }
//   }
//   // for(int i = 0 ; i < size;i++) {
//   //   imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
//   // }
// }
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  printf("11111\n");
  ros::init(argc, argv, "calibration");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber kinect_color_image = n.subscribe("rgb/image_raw", 1000, KinectColorCallback);
  ros::Subscriber kinect_depth_image = n.subscribe("depth_to_rgb/image_raw", 1000, KinectDepthCallback);
  // ros::Subscriber davis_color_image = n.subscribe("dvs/image_raw", 1000, DavisColorCallback);
  // ros::Subscriber davis_events = n.subscribe("dvs/events", 1000, DavisEventCallback);

  // ros::Subscriber kinect_color_image = n.subscribe("chatter1", 1000, chatterOneCallback);
  pangolin::CreateWindowAndBind("Main",1080,640);

  const int UI_WIDTH = 180;
  kinect_color_cam = pangolin::Display("kinect_color_cam")
    .SetAspect(640.0f/480.0f);
  kinect_depth_cam = pangolin::Display("kinect_depth_cam")
    .SetAspect(640.0f/480.0f);
  // davis_color_cam = pangolin::Display("davis_color_cam")
  //   .SetAspect(346.0f/260.0f);
  // davis_event_cam = pangolin::Display("davis_event_cam")
  //   .SetAspect(346.0f/260.0f);  
  // davis_event_cam = pangolin::Display("davis_event_cam")
  //   .SetAspect(346.0f/260.0f);

  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
  pangolin::Display("multi")
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
      .SetLayout(pangolin::LayoutEqual)
      .AddDisplay(kinect_color_cam)
      .AddDisplay(kinect_depth_cam);
      // .AddDisplay(davis_color_cam)
      // .AddDisplay(davis_event_cam);
      
  pangolin::Var<bool> start("ui.Start_Record",false,false);
  pangolin::Var<bool> stop("ui.Stop_Record",false,false);
  pangolin::Var<bool> save("ui.Save_Record",false,false);

  pangolin::Var<std::string>text("ui.Status", "Save 0 Images");
  pangolin::GlTexture imageTexture(kinect_color_width,kinect_color_height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
  pangolin::GlTexture depthTexture(kinect_color_width,kinect_color_height,GL_LUMINANCE,false,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);
  
  // pangolin::GlTexture DavisImageTexture(davis_color_width,davis_color_height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

  // pangolin::GlTexture DavisImageTexture(davis_color_width,davis_color_height,GL_LUMINANCE,false,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);




  printf("aaaaa\n");
  // while (true)
  // {
  //   glClear(GL_COLOR_BUFFER_BIT);
  //   if(pangolin::Pushed(start)){
  //     printf("click start\n");
  //     break;
  //   }
  //   pangolin::FinishFrame();
  // }
  printf("out\n");
  int save_count=0;

  time_t t;
  t=time(0);
  char now[64];
  struct tm *ttime;
  ttime = localtime(&t);
  strftime(now,64,"%Y-%m-%d-%H-%M-%S_",ttime); 
  std::string time_str=now;

  std::string kinect_file_path =  "/home/jiazhao/ros_code/catkin_calibration/data/stream_"+time_str+"/";


  // std::string kinect_file_path =  "/home/jiazhao/ros_code/catkin_calibration/data/kinect/";
  // std::string dvs_file_path =  "/home/jiazhao/ros_code/catkin_calibration/data/dvs/";

  std::string kinect_color_path =  kinect_file_path+"color/";
  std::string kinect_depth_path =  kinect_file_path+"depth/";
  // std::string davis_file_path =  "/home/jiazhao/ros_code/catkin_calibration/data/davis_eventmap/";
  
  // const int width =  64;
  // const int height = 48;


  while( !pangolin::ShouldQuit() ){
    glClear(GL_COLOR_BUFFER_BIT);
    if (!kinect_color.empty()){
      kinect_color_cam.Activate();
      imageTexture.Upload(kinect_color.data,GL_BGRA,GL_UNSIGNED_BYTE);
      imageTexture.RenderToViewportFlipY();
    }

    if (!kinect_depth.empty()){
      cv::Mat image_grayscale = kinect_depth.clone();
      image_grayscale.convertTo(image_grayscale, CV_8U, 256 / 5000.0);
      kinect_depth_cam.Activate();
      depthTexture.Upload(image_grayscale.data,GL_LUMINANCE,GL_UNSIGNED_BYTE);
      depthTexture.RenderToViewportFlipY();
    }

    // if(!davis_event.empty()){
    //   cv::imshow("davis_event",davis_event);
    //   cv::waitKey(3);
    // }

    // if(!davis_color.empty()){
    //   cv::imshow("davis_color",davis_color);
    //   cv::waitKey(3);
    // }

    // if (!davis_event.empty()){
    
    //   davis_event_cam.Activate();
   
    //   setImageData_withmat(imageArray,davis_color_width*davis_color_height,davis_event);
    //   cv::Mat img_test(davis_color_height,davis_color_width,CV_8UC1,imageArray);

    //   cv::imshow("aaa",img_test);
    //   cv::waitKey(3);

    //   pangolin::GlTexture DavisEventTexture(davis_color_height,davis_color_width,GL_LUMINANCE,false,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);

    //   std::cout<<davis_event.type()<<std::endl;

    //   printf("height:%d width%d",davis_event.size().height,davis_event.size().width);

    //   DavisEventTexture.Upload(img_test.data,GL_LUMINANCE,GL_UNSIGNED_BYTE);
    //   DavisEventTexture.RenderToViewportFlipY();

    //   }



    if( pangolin::Pushed(stop) ){
      return 0;
        // std::cout << "now save an image" << std::endl;
        // save_count++;
        // std::cout << save_count << std::endl;
        // cv::imwrite((depth_file_path+std::to_string(save_count)+"_depth.png").c_str(),kinect_depth);
        // cv::imwrite((color_file_path+std::to_string(save_count)+"_color.png").c_str(),kinect_color);
        // // cv::imwrite((dvs_file_path+std::to_string(save_count)+"_davis_color.png").c_str(),davis_color);
        // // cv::imwrite((dvs_file_path+std::to_string(save_count)+"_davis_event.png").c_str(),davis_event);

        // cv::Mat image_grayscale = kinect_depth.clone();
        // image_grayscale.convertTo(image_grayscale, CV_8U, 256 / 5000.0);
        // cv::imwrite((kinect_file_path+std::to_string(save_count)+"_depth_vis.png").c_str(),image_grayscale);
        // text.operator=("Save "+std::to_string(save_count)+" Images");
    }


    if( pangolin::Pushed(start) ){
        std::cout << "now start record" << std::endl;

        int isCreate0=mkdir(kinect_file_path.c_str(),S_IRUSR|S_IWUSR|S_IXUSR|S_IRWXG|S_IRWXO);
        int isCreate1=mkdir(kinect_color_path.c_str(),S_IRUSR|S_IWUSR|S_IXUSR|S_IRWXG|S_IRWXO);
        int isCreate2=mkdir(kinect_depth_path.c_str(),S_IRUSR|S_IWUSR|S_IXUSR|S_IRWXG|S_IRWXO);

        if(!isCreate0 || !isCreate1 || !isCreate2){
          printf("dir create failed");
        }

        record_streaming=true;
    }

    if( pangolin::Pushed(save) ){
        std::cout << "now stop record" << std::endl;
        record_streaming=false;
        break;
    }




    ros::spinOnce();
    pangolin::FinishFrame();

  }
  // std::string kinect_color_path =  "/home/jiazhao/ros_code/catkin_calibration/data/kinect_color/";
  // std::string kinect_depth_path =  "/home/jiazhao/ros_code/catkin_calibration/data/kinect_depth/";
  // std::string davis_file_path =  "/home/jiazhao/ros_code/catkin_calibration/data/davis_eventmap/";

  printf("write images\n");

  // printf("v_events_images size:%d \n",v_events_images.size());
  printf("v_color_images size:%d \n",v_color_images.size());
  printf("v_depth_images size:%d \n",v_depth_images.size());
  // printf("v_events size:%d \n",v_events.size());

  // for (int i=0;i<v_events_images.size();i++){
  //   printf("aaa\n");
  //   // std::stringstream timestr;

  //   // timestr << v_events_images[i].timestamp;
  //   // printf("%s \n",(davis_file_path+to_string(v_events_images[i].timestamp)+".png").c_str());
  //   cv::imwrite((davis_file_path+to_string(v_events_images[i].timestamp.count())+".png").c_str(),v_events_images[i].map);
  //   printf("write event images %d \n",i);
  // }

  for (int i=0;i<v_color_images.size();i++){
    // std::stringstream timestr;
    // timestr << v_events_images[i].timestamp;
    cv::imwrite((kinect_color_path+to_string(v_color_images[i].timestamp.count())+".png").c_str(),v_color_images[i].map);
    printf("write color images %d \n",i);

  }

  for (int i=0;i<v_depth_images.size();i++){
    // std::stringstream timestr;
    // timestr << v_events_images[i].timestamp;
    cv::imwrite((kinect_depth_path+to_string(v_depth_images[i].timestamp.count())+".png").c_str(),v_depth_images[i].map);
    printf("write depth images %d \n",i);
  }
  // ofstream ofile;
  // ofile.open((davis_file_path+"events.txt").c_str(), ios::out);//②
  // for(int i=0;i<v_events.size();i++){
  //   ofile<<to_string(v_events[i].timestamp.count())+" "+to_string(v_events[i].x)+" "+to_string(v_events[i].y)+" "+to_string(v_events[i].p)+"\n";
  // }
  // ofile.close();

  return 0;
}