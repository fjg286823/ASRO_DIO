#include <rosefusion.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <ctime>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pangolin/pangolin.h>
#include "sensor_msgs/Imu.h"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>


#include <thread>
#include <mutex>
#include <condition_variable>


#include <chrono>
#include <csignal>
using namespace std;
using namespace std::chrono;



std::condition_variable con;
std::queue<imu_data> imu_buf;
std::queue<img_data> depth_buf;
std::queue<img_data> color_buf;

int sum_of_wait = 0;
double current_time = -1;

std::mutex m_buf;
std::mutex m_state;
// std::mutex i_buf;
std::mutex m_estimator;



//pangolin 

pangolin::View color_cam;
pangolin::View shadded_cam; 
pangolin::View depth_cam; 

rosefusion::Pipeline * pipeline;
image_transport::ImageTransport * it;
ros::NodeHandle * n;


std::atomic<bool> stop(false);

void sigint_handler(int sig) {
    if (sig == SIGINT) {
        std::cout << "[Main] DWIO terminated!\n";
        stop = true;
    }
}

double latest_time;

double last_imu_t = 0;


std::vector<std::string> v_index; //保存所有被处理的图像时间戳

//订阅话题，并保存到对应buf,随后唤醒线程！

void KinectColorCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("receive color map ");


    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat color_img = cv_ptr->image;

    //加一个转换函数转成640*480

    img_data id = img_data();
    id.time=msg->header.stamp.toSec();;
    id.img = color_img;

    m_buf.lock();
    color_buf.push(id);
    m_buf.unlock();
    con.notify_one();


}



void KinectDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    // ROS_INFO("receive depth map ");


    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_img = cv_ptr->image;

    img_data id = img_data();
    id.time=msg->header.stamp.toSec();
    id.img = depth_img;

    m_buf.lock();
    depth_buf.push(id);
    m_buf.unlock();
    con.notify_one();


}

void inertialCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("receive  imu ");


    imu_data id;
    id.time=msg->header.stamp.toSec();
    id.rx=msg->angular_velocity.x;
    id.ry=msg->angular_velocity.y;
    id.rz=msg->angular_velocity.z;

    id.dx=msg->linear_acceleration.x;
    id.dy=msg->linear_acceleration.y;
    id.dz=msg->linear_acceleration.z;
    
    if (id.time <= last_imu_t){
        ROS_WARN("imu message in disorder!");
        return;   
    }

    last_imu_t = id.time;

    m_buf.lock();
    imu_buf.push(id);
    m_buf.unlock();
    con.notify_one();
  

}

//只采样8个imu数据应该是为了减少计算！
std::vector<imu_data> sample_imu(std::vector<imu_data> v_id ,int sample_size)
{
    if (v_id.size()<=sample_size){
        return v_id;
    }

    std::vector<imu_data> v_id_sample;;
    double gap_size = int((v_id.size() - 2) / (sample_size - 2));

    v_id_sample.push_back(v_id[0]);

    for (int i=gap_size; i<v_id.size()-1; i+=gap_size ){
        v_id_sample.push_back(v_id[i]);
    }

    v_id_sample.push_back(v_id[v_id.size()-1]);
    return v_id_sample;

    


}

// std::vector<std::tuple<img_data, img_data, std::vector<imu_data>>>
// getMeasurements()
// {
//     std::vector<std::tuple<img_data, img_data, std::vector<imu_data>>> measurements;

//     while (true)
//     {
//         if (imu_buf.empty() || depth_buf.empty() || color_buf.empty())
//             return measurements;

//         if ((imu_buf.back().time <= depth_buf.front().time ))
//         {
//             ROS_WARN("wait for imu, only should happen at the beginning");
//             sum_of_wait++;
//             return measurements;
//         }

//         if ((imu_buf.front().time >= depth_buf.front().time ))
//         {
//             ROS_WARN("throw img, only should happen at the beginning");
//             depth_buf.pop();
//             color_buf.pop();
//             continue;
//         }

//         img_data depth_data = depth_buf.front();
//         img_data color_data = color_buf.front();

//         depth_buf.pop();
//         color_buf.pop();

//         std::vector<imu_data> IMUs;
//         while (imu_buf.front().time < depth_data.time )
//         {
//             IMUs.emplace_back(imu_buf.front());
//             imu_buf.pop();
//         }
//         // IMUs.emplace_back(imu_buf.front());
//         if (IMUs.empty()){
//             ROS_WARN("no imu between two image");
//         }
//         measurements.emplace_back( depth_data, color_data, IMUs);
//     }
//     return measurements;
// }


// std::vector<std::tuple<img_data, img_data, std::vector<imu_data>>>
// getMeasurements()
// {
//     std::vector<std::tuple<img_data, img_data, std::vector<imu_data>>> measurements;

//     while (true)
//     {
//         if (imu_buf.empty() || depth_buf.empty() || color_buf.empty())
//             return measurements;

//         if ((imu_buf.back().time <= depth_buf.front().time ))
//         {
//             ROS_WARN("wait for imu, only should happen at the beginning");
//             sum_of_wait++;
//             return measurements;
//         }

//         if ((imu_buf.front().time >= depth_buf.front().time ))
//         {
//             ROS_WARN("throw img, only should happen at the beginning");
//             depth_buf.pop();
//             color_buf.pop();
//             continue;
//         }

//         //only need newest img_data
//         img_data depth_data = depth_buf.back();
//         img_data color_data = color_buf.back();
//         depth_buf.pop();
//         color_buf.pop();

//         std::vector<imu_data> IMUs;
//         while (imu_buf.front().time < depth_data.time )
//         {
//             IMUs.emplace_back(imu_buf.front());
//             imu_buf.pop();
//         }
//         // IMUs.emplace_back(imu_buf.front());
//         if (IMUs.empty()){
//             ROS_WARN("no imu between two image");
//         }
//         measurements.emplace_back( depth_data, color_data, IMUs);
//     }
//     return measurements;
// }


std::vector<std::tuple<img_data, img_data, std::vector<imu_data>>>
getMeasurements()
{
    std::vector<std::tuple<img_data, img_data, std::vector<imu_data>>> measurements;

    while (true)
    {
        if (imu_buf.empty() || depth_buf.empty() || color_buf.empty())
            return measurements;

        if ((imu_buf.back().time <= depth_buf.front().time ))
        {
            ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if ((imu_buf.front().time >= depth_buf.back().time ))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            while(!color_buf.empty()){
                color_buf.pop();
            }
            while(!depth_buf.empty()){
                depth_buf.pop();
            }
            continue;
        }

        //only need newest img_data
        img_data depth_data = depth_buf.back();
        img_data color_data = color_buf.back();
        depth_buf.pop();
        color_buf.pop();

        std::vector<imu_data> IMUs;
        while (imu_buf.front().time < depth_data.time )
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        // IMUs.emplace_back(imu_buf.front()); 这里应该就是不想做插值了
        if (IMUs.empty()){
            ROS_WARN("no imu between two image");
        }
        measurements.emplace_back( depth_data, color_data, IMUs);
    }
    return measurements; //得到的是上一帧到当前帧的imu数据(小于当前帧）和图像
}


void process_fusion()
{

    std::cout<<"aaaaa\n";

    rosefusion::CameraParameters camera_config = pipeline->camera_parameters;
    rosefusion::ControllerConfiguration controller_config = pipeline->controller_config;
    std::cout<<"bbbbb\n";
    //init Pangolin ================================

    if (controller_config.render_surface){
        pangolin::CreateWindowAndBind("Main",2880,1440);
        const int UI_WIDTH = 180;
        color_cam = pangolin::Display("color_cam")
            .SetAspect((float)camera_config.image_width/(float)camera_config.image_height);
        shadded_cam = pangolin::Display("shadded_cam")
            .SetAspect((float)camera_config.image_width/(float)camera_config.image_height);
        depth_cam = pangolin::Display("depth_cam")
            .SetAspect((float)camera_config.image_width/(float)camera_config.image_height);


        pangolin::Display("shaded")
            .SetBounds(0.0, 1.0, 0.0,2.0f/3.0 )
            .AddDisplay(shadded_cam);

        pangolin::Display("kinect_color")
            .SetBounds(0.5, 1.0, 2.0f/3, 1.0)
            .AddDisplay(color_cam);
            
        pangolin::Display("kinect_depth")
            .SetBounds(0.0, 0.5, 2.0f/3, 1.0)
            .AddDisplay(depth_cam);
    }
    pangolin::GlTexture imageTexture=pangolin::GlTexture(camera_config.image_width,camera_config.image_height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture shadTexture=pangolin::GlTexture(camera_config.image_width,camera_config.image_height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture depthTexture=pangolin::GlTexture(camera_config.image_width,camera_config.image_height,GL_LUMINANCE,false,0,GL_LUMINANCE,GL_UNSIGNED_BYTE);
    std::cout<<"xxxxxx\n";

    cv::Mat shaded_img(camera_config.image_height, camera_config.image_width,CV_8UC3);
    cv::Mat depth_vis(camera_config.image_height, camera_config.image_width,CV_8U);

    image_transport::Publisher shaded_pub = it->advertise("/shaded_image/image_raw",1);
    ros::Publisher pub_path = n->advertise<nav_msgs::Path>("/dio/trajectory",1000);
    ros::Publisher pub_odometry = n->advertise<nav_msgs::Odometry>("/dio/odometry",1000);
    nav_msgs::Path path;

    std::cout<<"ccccccc\n";
    int n_imgs=0;
    while(ros::ok()&&!stop)
    {
        // std::cout<<"add lock\n";
        std::vector<std::tuple<img_data, img_data, std::vector<imu_data>>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        std::cout<<"read measurements\n";
        //如果无法得到一帧打包好的数据就等待！
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;//得到的是上一帧到当前帧的imu数据(小于当前帧）和图像
                 });
        lk.unlock();

        m_estimator.lock(); //这个锁感觉不是很有意思！
        for (auto &measurement : measurements )
        {

            clock_t time_stt=clock();
            std::cout << "------------------------------------------------------------------" << std::endl;
            // std::cout<<"new frame\n";
            std::vector<imu_data> v_imu = std::get<2>(measurement);
            img_data depth_data = std::get<0>(measurement);
            img_data color_data = std::get<1>(measurement);
            v_index.push_back(std::to_string(depth_data.time));


            printf("detph time: %f\n",depth_data.time);
            printf("color time: %f\n",color_data.time);

            if (v_imu.size()==0){ //如果发生这种情况应该是出问题了，因为不可能到这里
                exit(0);
            }
            if (n_imgs != 0){
                std::vector<imu_data> v_imu_sample = sample_imu(v_imu,8);//只采样8个imu数据应该是为了减少计算！

                for (auto &id : v_imu_sample){
                    // std::cout<<id.time<<" "<<id.rx<<" "<<id.ry<<" "<<id.rz<<" "<<id.dx<<" "<<id.dy<<" "<<id.dz<<"\n";
                    pipeline->add_imu(id); //将数据存放到pipeline.imu_model.v_imu
                }
            }
            pipeline->set_timegap(0.03);
            std::cout<<"update frame\n";
            // bool success = pipeline->process_frame(depth_data.img, color_data.img, shaded_img);
            bool success = pipeline->process_frame_hash(depth_data.img, color_data.img, shaded_img);

            //发布tracking轨迹的topic，若不需要则注释
            Eigen::Matrix4d newest_pose = pipeline->return_pose();
            //pub camera pose
            nav_msgs::Odometry odometry;
            std_msgs::Header header;
            header.frame_id ="world";
            // header.stamp.sec = int(depth_data.time);
            // header.stamp.nsec = (depth_data.time-int(depth_data.time))*100000;
            header.stamp = ros::Time::now();
            odometry.child_frame_id = "world";
            Eigen::Quaterniond tmp_Q;
            Eigen::Matrix3d tmp_rotation=newest_pose.block(0,0,3,3);

            tmp_Q = Eigen::Quaterniond(tmp_rotation);
            odometry.pose.pose.position.x = newest_pose(0,3)/1000;
            odometry.pose.pose.position.y = newest_pose(1,3)/1000;
            odometry.pose.pose.position.z = newest_pose(2,3)/1000;

            odometry.pose.pose.orientation.x = tmp_Q.x();
            odometry.pose.pose.orientation.y = tmp_Q.y();
            odometry.pose.pose.orientation.z = tmp_Q.z();
            odometry.pose.pose.orientation.w = tmp_Q.w();
            pub_odometry.publish(odometry);

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header =header;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose = odometry.pose.pose;
            path.header = header;
            path.header.frame_id = "world";
            path.poses.push_back(pose_stamped);
            pub_path.publish(path);
            //end 发布tracking轨迹的topic，若不需要则注释




            color_cam.Activate();
            imageTexture.Upload(color_data.img.data,GL_BGR,GL_UNSIGNED_BYTE);
            imageTexture.RenderToViewportFlipY();

            depth_cam.Activate();
            depth_data.img.convertTo(depth_vis,CV_8U,256/5000.0);
            depthTexture.Upload(depth_vis.data,GL_LUMINANCE,GL_UNSIGNED_BYTE);
            depthTexture.RenderToViewportFlipY();
            if (success){
                std::cout<<"update pangolin\n";
                shadded_cam.Activate();
                shadTexture.Upload(shaded_img.data,GL_BGR,GL_UNSIGNED_BYTE);
                shadTexture.RenderToViewportFlipY();       
                sensor_msgs::ImagePtr rgb_image= cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGR8,shaded_img).toImageMsg();
                shaded_pub.publish(rgb_image);
            }
            pangolin::FinishFrame();
            // if (n_imgs==50){
            //     exit(0);
            // }
            n_imgs++;
            std::cout <<"time per frame="<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<std::endl;

        }
        m_estimator.unlock();
        // if (n_imgs%30==0){
        //     pipeline->get_poses(v_index);
        // }


    }
    // std::cout <<"time per frame="<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC/n_imgs<<"ms"<<std::endl;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rogio_threads");
    signal(SIGINT, sigint_handler);
    n = new ros::NodeHandle();
    // init parameters ==============================
    // std::string config_file;
    // std::string config_file="/home/jiazhao/workspace/ROSEFusion/src/ROGIO-ROS-threads/config/Kinect_config.yaml";
    std::string config_file="/home/fjg/work/catkin_dio/catkin_dio/src/ROGIO-ROS-threads/config/Astra2_config.yaml";



    n->param("config_file", config_file, config_file); //用于定义相机的参数和imuh和图像话题以及外参、内参！

    // n.getParam("/rogio/config_file",config_file); 

    std::cout<< "load path:"  << config_file  << "\n";

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }


    rosefusion::CameraParameters camera_config(fsSettings);
    rosefusion::DataConfiguration data_config(fsSettings);
    rosefusion::ControllerConfiguration controller_config(fsSettings);

    Eigen::Matrix4d cali_imu_to_cam_transformation = Eigen::Matrix4d::Identity();;
    cv::Mat cv_R, cv_T;
    fsSettings["extrinsicRotation"] >> cv_R;
    fsSettings["extrinsicTranslation"] >> cv_T;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);
    cali_imu_to_cam_transformation.block(0,0,3,3)=eigen_R;
    cali_imu_to_cam_transformation.block(0,3,3,1)=eigen_T;
    rosefusion::IMU_Model imu_model(cali_imu_to_cam_transformation); //初始化外参，以及零偏、速度、加速度

    // std::cout<<"Init ROSEFusion"<<std::endl;
    pipeline = new rosefusion::Pipeline(camera_config, data_config, controller_config, imu_model);

    std::string depth_topic = fsSettings["depth_topic"];
    std::string image_topic = fsSettings["image_topic"];
    std::string imu_topic = fsSettings["imu_topic"];

    //init ros =============================
    std::cout<<"depth topic: "<<depth_topic<<std::endl;
    std::cout<<"image topic: "<<image_topic<<std::endl;
    std::cout<<"imu topic: "<<imu_topic<<std::endl;


    it =new image_transport::ImageTransport(*n);

    //(1) subscribe compressed image topic and depth topic (for shiyanshi)
    // image_transport::Subscriber depth_sub = it->subscribe(depth_topic,1,&KinectDepthCallback,image_transport::TransportHints("compressedDepth"));
    // image_transport::Subscriber color_sub = it->subscribe(image_topic,1,&KinectColorCallback,image_transport::TransportHints("compressed"));
    //(2) subscribe raw image topic and depth topic (for kaile)
    ros::Subscriber depth_sub = n->subscribe(depth_topic, 1, &KinectDepthCallback);
    ros::Subscriber color_sub = n->subscribe(image_topic, 1, &KinectColorCallback);
    
    ros::Subscriber sub = n->subscribe(imu_topic, 1000, inertialCallback);

    // shaded_pub = new it.advertise("/shaded_image/image_raw",1);


    // cv::Mat shaded_img(camera_config.image_height, camera_config.image_width,CV_8UC3);
    // kinect_color = cv::Mat (camera_config.image_height, camera_config.image_width, CV_8UC3, cv::Scalar(0, 0, 0));
    // kinect_depth = cv::Mat (camera_config.image_height, camera_config.image_width, CV_16UC1, cv::Scalar(0, 0, 0));


        

    //prepare running =========================================================
    // int n_imgs=0;
    // ros::Rate r(25);
    // clock_t time_stt=clock();
    // bool success=false;
    // double sec_current_depth=0;
    // double t0=0;

    std::cout<<"begin tracking\n"<<std::endl;

    std::thread measurement_process{process_fusion};
    // if(ros::ok()){
    ros::spin();
    // }



    // printf("images %d\n",n_imgs);
    // std::cout <<"time per frame="<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC/n_imgs<<"ms"<<std::endl;

    if (controller_config.save_trajectory){
        pipeline->get_poses(v_index);
    }


    auto points = pipeline->extract_pointcloud();
    rosefusion::export_ply(data_config.result_path+data_config.seq_name+"_points.ply",points);


    return 0;
}
