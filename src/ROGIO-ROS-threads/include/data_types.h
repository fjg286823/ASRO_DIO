#ifndef ROSEFUSION_DATA_TYPES_H
#define ROSEFUSION_DATA_TYPES_H

// #if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Weffc++"
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <Eigen/Eigen>
#include <time.h>
#include <iostream>
#include <fstream>
#include <queue>

// #else
// #include <cuda_runtime.h>
// #include <opencv2/core/cuda.hpp>
// #include <Eigen/Dense>
// #endif

using cv::cuda::GpuMat;

struct imu_data
{
    double time;
    double dx;
    double dy;
    double dz;
    double rx;
    double ry;
    double rz;
};

struct img_data
{
    double time;
    cv::Mat img;
};

namespace rosefusion
{

    struct CameraParameters
    {
        int image_width{0};
        int image_height{0};
        float focal_x{.0};
        float focal_y{.0};
        float principal_x{.0};
        float principal_y{.0};
        Eigen::Matrix4d extrinic_d_2_c;


        CameraParameters(const cv::FileStorage fsSettings)
        {
            fsSettings["fx"] >> focal_x;
            fsSettings["fy"] >> focal_y;
            fsSettings["cx"] >> principal_x;
            fsSettings["cy"] >> principal_y;

            std::cout << "fx: " << focal_x << "fy: " << focal_x << "cx: " << principal_x << "cy: " << principal_y << std::endl;
            fsSettings["image_width"] >> image_width;
            fsSettings["image_height"] >> image_height;

            std::cout << "image_width: " << image_width << "image_height: " << image_height << std::endl;

            extrinic_d_2_c = Eigen::Matrix4d::Identity();
        }
    };

    struct PointCloud
    {
        cv::Mat vertices;
        cv::Mat normals;
        cv::Mat color;
        int num_points;
    };

    struct interval_frame
    {
        std::vector<imu_data> v_imu;
        Eigen::Vector3d v;
        Eigen::Matrix3d w_2_imu_rotation;
        Eigen::Matrix4d cam_pose;
        Eigen::Vector3d imu_translation;
        double time_gap;
        double weight = 0;
    };

    struct IMU_Model
    {

        Eigen::Vector3d imu_residual_rotation;
        Eigen::Vector3d acc_frame_measuerment;

        Eigen::Matrix4d imu_2_cam;
        Eigen::Matrix3d imu_2_cam_rotation;
        Eigen::Vector3d imu_2_cam_translation;

        double last_time_gap;
        Eigen::Vector3d last_velocity;
        Eigen::Vector3d last_acc;

        double current_time_gap;
        Eigen::Vector3d current_velocity;
        Eigen::Vector3d current_acc;

        std::vector<imu_data> v_imu; // 存放两帧之间的imu数据

        Eigen::Vector3d g;
        double g_weight;
        Eigen::Vector3d latest_v;

        std::vector<interval_frame> v_iframes;

        Eigen::Vector3d bias_acc;
        Eigen::Vector3d bias_gyr;

        IMU_Model(const Eigen::Matrix4d &cali_imu_2_cam)
        {

            imu_2_cam = cali_imu_2_cam;
            imu_2_cam_rotation = cali_imu_2_cam.block(0, 0, 3, 3);
            imu_2_cam_translation = cali_imu_2_cam.block(0, 3, 3, 1);

            bias_acc << 0, 0, 0;
            bias_gyr << 0, 0, 0;

            g << 0, 0, 0;
            g_weight = 0;

            imu_residual_rotation << 0, 0, 0;

            last_velocity << 0, 0, 0;
            last_acc << 0, 0, 0;

            current_velocity << 0, 0, 0;
            current_acc << 0, 0, 0;
        }

        Eigen::Quaterniond deltaQ(const Eigen::Vector3d &theta)
        {
            // typedef typename Derived::Scalar Scalar_t;

            Eigen::Quaternion<double> dq;
            Eigen::Matrix<double, 3, 1> half_theta = theta;
            half_theta /= static_cast<double>(2.0);
            dq.w() = static_cast<double>(1.0);
            dq.x() = half_theta.x();
            dq.y() = half_theta.y();
            dq.z() = half_theta.z();
            return dq;
        }

        void fastPredictIMURotation(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity, Eigen::Quaterniond &latest_Q,
                                    Eigen::Vector3d &latest_acc, Eigen::Vector3d &latest_gyr, double *latest_time)
        {

            double dt = t - *latest_time;

            Eigen::Vector3d un_gyr = 0.5 * (latest_gyr + angular_velocity);

            latest_Q = latest_Q * deltaQ(un_gyr * dt);

            latest_acc = linear_acceleration;
            latest_gyr = angular_velocity;
            *latest_time = t;
        }

        void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity,
                            double *latest_time, Eigen::Vector3d &latest_acc, Eigen::Vector3d &latest_gyr, Eigen::Vector3d &latest_P,
                            Eigen::Quaterniond &latest_Q, Eigen::Vector3d &latest_V, const Eigen::Vector3d &g)
        {
            double dt = t - *latest_time;
            Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc)-g;
            Eigen::Vector3d un_gyr = 0.5 * (latest_gyr + angular_velocity);

            latest_Q = latest_Q * deltaQ(un_gyr * dt);

            Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration)-g;
            Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

            latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
            latest_V = latest_V + dt * un_acc;
            latest_acc = linear_acceleration;
            latest_gyr = angular_velocity;
            *latest_time = t;
        }

        void fastPredictIMUVelocity(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity,
                                    double *latest_time, Eigen::Vector3d &latest_acc, Eigen::Vector3d &latest_gyr,
                                    Eigen::Quaterniond &latest_Q, Eigen::Vector3d &latest_V, const Eigen::Vector3d &g)
        {
            double dt = t - *latest_time;
            *latest_time = t;
            Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc)-g;
            Eigen::Vector3d un_gyr = 0.5 * (latest_gyr + angular_velocity);

            latest_Q = latest_Q * deltaQ(un_gyr * dt);

            Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration)-g;
            Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            latest_V = latest_V + dt * un_acc;
            latest_acc = linear_acceleration;
            latest_gyr = angular_velocity;
        }

        void update_gravity(Eigen::Vector3d new_g, double error)
        {

            double new_weight = (1 - error) * (1 - error);
            g = (g_weight * g + new_weight * new_g) / (g_weight + new_weight);
            g_weight = new_weight + g_weight;
        }

        // Eigen::Matrix4d inverse_trans(Eigen::Matrix4d& trans){
        //     Eigen::Matrix4d inv_trans;
        //     inv_trans.block(0,0,3,3)=trans.block(0,0,3,3).transpose();
        //     inv_trans.block(0,3,3,1)=-trans.block(0,0,3,3).transpose()*trans.block(0,3,3,1);
        //     inv_trans.row(3)<<0,0,0,1;
        //     return inv_trans;

        // }

        // struct interval_frame{
        //     std::vector<imu_data>  v_imu;
        //     Eigen::Vector3d v;
        //     Eigen::Matrix3d w_2_imu;
        //     Eigen::Vector3d imu_translation;
        //     double time_gap;
        // }

        void update_frame(Eigen::Matrix3d w_2_imu_rotation, Eigen::Vector3d imu_translation, Eigen::Matrix4d cam_pose)
        {
            last_time_gap = current_time_gap;
            interval_frame iframe;
            // iframe.v= v;
            iframe.w_2_imu_rotation = w_2_imu_rotation; //世界到第i帧imu的旋转矩阵
            iframe.imu_translation = imu_translation; //i->i+1的imu平移
            iframe.time_gap = current_time_gap;
            iframe.v_imu = v_imu;
            iframe.cam_pose = cam_pose;//上一帧位姿

            v_iframes.push_back(iframe);
            v_imu.clear();
            int window_size = 3;

            if (v_iframes.size() > window_size)
            {
                v_iframes.erase(v_iframes.begin());
            }

            // std::cout<<"v_iframes size:\n"<<v_iframes.size();

            // for (int i=0; i< v_iframes.size();i++){
            //     std::cout<<"imu data size:"<<v_iframes[i].v_imu.size()<<"\n";
            // }
            // exit(0);
        }

        void update_velocity()
        {

            for (int i = 0; i < v_iframes.size(); i++)
            {

                interval_frame iframe = v_iframes[i];

                Eigen::Vector3d imu_v = iframe.v;
                Eigen::Vector3d imu_g = iframe.w_2_imu_rotation * g;
                std::cout << "iframe init v:\n"
                          << imu_v << std::endl;
                // std::cout<<"iframe init g:\n"<<imu_g<<std::endl;
                Eigen::Quaterniond relative_q(1, 0, 0, 0);
                Eigen::Vector3d latest_acc(0, 0, 0);
                Eigen::Vector3d latest_gyr(0, 0, 0);
                Eigen::Vector3d latest_P(0, 0, 0);
                double latest_time;

                for (int j = 0; j < iframe.v_imu.size(); j++)
                {
                    imu_data id = iframe.v_imu[j];
                    Eigen::Vector3d acc(id.dx, id.dy, id.dz);
                    Eigen::Vector3d gyr(id.rx, id.ry, id.rz);
                    acc = acc - bias_acc;
                    gyr = gyr - bias_gyr;

                    if (j != 0)
                    {
                        fastPredictIMU(id.time, acc, gyr, &latest_time, latest_acc, latest_gyr, latest_P, relative_q, imu_v, imu_g); //更新v
                    }
                    else
                    {
                        latest_acc = acc;
                        latest_gyr = gyr;
                        latest_time = id.time;
                    }
                }

                // std::cout<<"imu_v:\n"<<imu_v<<std::endl;
                // std::cout<<"predict imu translation:\n"<<latest_P<<std::endl;
                if ((i + 1) < v_iframes.size())
                {
                    // std::cout<<"aaaaa\n";
                    v_iframes[i + 1].v = imu_v;
                }
                else
                {
                    // std::cout<<"bbbbb\n";
                    latest_v = imu_v;
                }

                Eigen::Matrix4d imu_realtive_motion = Eigen::Matrix4d::Identity();
                imu_realtive_motion.block(0, 0, 3, 3) = relative_q.toRotationMatrix();
                imu_realtive_motion.block(0, 3, 3, 1) = latest_P;

                Eigen::Matrix4d cam_relative_motion = imu_2_cam * imu_realtive_motion * inverse_trans(imu_2_cam);

                Eigen::Vector3d global_trans = iframe.cam_pose.block(0, 0, 3, 3) * cam_relative_motion.block(0, 3, 3, 1);
                // std::cout<<"predict global translation_"<<i<<"\n"<<global_trans<<std::endl;
            }
        }

        Eigen::Matrix4d integration_relative_rotation(Eigen::Matrix4d current_pose)
        {
            Eigen::Quaterniond relative_q(1, 0, 0, 0);
            Eigen::Vector3d latest_acc(0, 0, 0);
            Eigen::Vector3d latest_gyr(0, 0, 0);
            double latest_time;
            for (int i = 0; i < v_imu.size(); i++)
            {
                imu_data id = v_imu[i];
                Eigen::Vector3d acc(id.dx, id.dy, id.dz);
                Eigen::Vector3d gyr(id.rx, id.ry, id.rz);
                gyr = gyr - bias_gyr;
                acc = acc - bias_acc;

                if (i != 0)
                {
                    fastPredictIMURotation(id.time, acc, gyr, relative_q, latest_acc, latest_gyr, &latest_time);
                }
                else
                {
                    latest_acc = acc;
                    latest_gyr = gyr;
                    latest_time = id.time;
                }
            }

            // double interval_angle;
            // Eigen::Vector3d rotation_axis=quaternion_2_axisangle(relative_q,&interval_angle);
            // rotation_axis=imu_2_cam_rotation.transpose() * rotation_axis;
            // relative_q=axisangle_2_quaternion(rotation_axis,interval_angle);
            // std::cout<<"relative q"<<relative_q.x()<<" "<<relative_q.y()<<std::endl;
            // relative_q=relative_q.normalized();
            // Eigen::Matrix3d imu_2_w_rotation=current_pose.block(0,0,3,3).transpose() * imu_2_cam_rotation;
            // std::cout<<"imu_2_w_rotation"<<imu_2_w_rotation<<std::endl;

            Eigen::Matrix3d cam_relative_rotation = imu_2_cam_rotation * relative_q.toRotationMatrix() * imu_2_cam_rotation.transpose();
            // Eigen::Matrix3d w_relative_rotation=imu_2_w_rotation*relative_q.toRotationMatrix()*imu_2_w_rotation.transpose();

            // std::cout<<"cam_relative_rotation"<<cam_relative_rotation<<std::endl;
            // std::cout<<"w_relative_rotation"<<w_relative_rotation<<std::endl;

            Eigen::Matrix4d cam_relative_motion = Eigen::Matrix4d::Identity();
            // cam_relative_motion.block(0,0,3,3)= w_relative_rotation;
            cam_relative_motion.block(0, 0, 3, 3) = cam_relative_rotation;

            return cam_relative_motion;
        }

        // void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity,
        //     double *latest_time, Eigen::Vector3d &latest_acc, Eigen::Vector3d &latest_gyr, Eigen::Vector3d &latest_P,
        //     Eigen::Quaterniond& latest_Q, Eigen::Vector3d &latest_V, const Eigen::Vector3d &g)
        //     {

        Eigen::Matrix4d inverse_trans(Eigen::Matrix4d &trans)
        {
            Eigen::Matrix4d inv_trans;
            inv_trans.block(0, 0, 3, 3) = trans.block(0, 0, 3, 3).transpose();
            inv_trans.block(0, 3, 3, 1) = -trans.block(0, 0, 3, 3).transpose() * trans.block(0, 3, 3, 1);
            inv_trans.row(3) << 0, 0, 0, 1;
            return inv_trans;
        }

        // void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity,
        //     double *latest_time, Eigen::Vector3d &latest_acc, Eigen::Vector3d &latest_gyr, Eigen::Vector3d &latest_P,
        //     Eigen::Quaterniond& latest_Q, Eigen::Vector3d &latest_V, const Eigen::Vector3d &g)

        Eigen::Matrix4d integration_relative_motion(Eigen::Matrix4d current_pose)
        {

            Eigen::Quaterniond relative_q(1, 0, 0, 0);
            Eigen::Vector3d latest_acc(0, 0, 0);
            Eigen::Vector3d latest_gyr(0, 0, 0);
            Eigen::Vector3d latest_P(0, 0, 0);

            Eigen::Matrix4d w_2_imu = inverse_trans(imu_2_cam) * inverse_trans(current_pose);

            // Eigen::Matrix3d current_rotation=current_pose.block(0,0,3,3);

            Eigen::Vector3d imu_v = latest_v;
            Eigen::Vector3d imu_g = w_2_imu.block(0, 0, 3, 3) * g;
            double latest_time;
            for (int i = 0; i < v_imu.size(); i++)
            {
                imu_data id = v_imu[i];
                Eigen::Vector3d acc(id.dx, id.dy, id.dz);
                Eigen::Vector3d gyr(id.rx, id.ry, id.rz);
                gyr = gyr - bias_gyr;
                acc = acc - bias_acc;

                if (i != 0)
                {
                    fastPredictIMU(id.time, acc, gyr, &latest_time, latest_acc, latest_gyr, latest_P, relative_q, imu_v, imu_g);
                    // fastPredictIMURotation(id.time,acc,gyr,relative_q,latest_acc,latest_gyr,&latest_time);
                }
                else
                {
                    latest_acc = acc;
                    latest_gyr = gyr;
                    latest_time = id.time;
                }
            }

            std::cout << "============ Predict new trans and rotation ==============\n";

            // Eigen::Matrix3d cam_relative_rotation=imu_2_cam_rotation*relative_q.toRotationMatrix()*imu_2_cam_rotation.transpose();

            // Eigen::Matrix4d cam_relative_motion=Eigen::Matrix4d::Identity();
            // cam_relative_motion.block(0,0,3,3)=cam_relative_rotation;
            // std::cout<<"predict new gloabl rotation2:\n"<< cam_relative_motion.block(0,0,3,3) <<std::endl;

            Eigen::Matrix4d imu_realtive_motion = Eigen::Matrix4d::Identity();
            imu_realtive_motion.block(0, 0, 3, 3) = relative_q.toRotationMatrix();
            imu_realtive_motion.block(0, 3, 3, 1) = latest_P;
            // std::cout<<"perdict new imu translation\n"<<latest_P<<std::endl;
            // std::cout<<"imu_realtive_motion:"<<imu_realtive_motion<<std::endl;
            // std::cout<<"imu_2_cam:"<<imu_2_cam<<std::endl;
            // std::cout<<"imu_2_cam_inverse:"<<inverse_trans(imu_2_cam)<<std::endl;

            Eigen::Matrix4d cam_relative_motion = imu_2_cam * imu_realtive_motion * inverse_trans(imu_2_cam);
            Eigen::Vector3d global_trans = current_pose.block(0, 0, 3, 3) * cam_relative_motion.block(0, 3, 3, 1);

            // std::cout<<"predict new gloabl trans:\n"<< global_trans <<std::endl;
            // std::cout<<"predict new gloabl rotation:\n"<< cam_relative_motion.block(0,0,3,3) <<std::endl;

            cam_relative_motion.block(0, 3, 3, 1) = global_trans;
            // cam_relative_motion.block(0,3,3,1)<<0,0,0;

            return cam_relative_motion;
        }
    };

    struct ControllerConfiguration
    {
        int max_iteration{20};
        std::string PST_path{"~"};
        float scaling_coefficient1{0.12};
        float scaling_coefficient2{0.12};
        float init_fitness{0.5};
        float momentum{0.9};
        bool scaling_inherit_directly{false};
        bool save_trajectory{false};
        bool save_scene{false};
        bool render_surface{false};

        ControllerConfiguration(const cv::FileStorage fsSettings)
        {
            fsSettings["PST_path"] >> PST_path;
            max_iteration = fsSettings["max_iteration"];
            scaling_coefficient1 = fsSettings["scaling_coefficient1"];
            scaling_coefficient2 = fsSettings["scaling_coefficient2"];
            init_fitness = fsSettings["init_fitness"];
            momentum = fsSettings["momentum"];

            scaling_inherit_directly = bool(int(fsSettings["scaling_inherit_directly"]));
            save_trajectory = bool(int(fsSettings["save_trajectory"]));
            save_scene = bool(int(fsSettings["save_scene"]));
            render_surface = bool(int(fsSettings["render_surface"]));
        }
    };

    struct DataConfiguration
    {

        int3 volume_size{make_int3(812, 512, 812)};

        float voxel_scale{30.f};

        float3 init_pos{volume_size.x / 2 * voxel_scale, volume_size.x / 2 * voxel_scale, volume_size.x / 2 * voxel_scale};

        float truncation_distance{120.f};

        float depth_cutoff_distance{8000.f};

        int triangles_buffer_size{3 * 2000000};

        int pointcloud_buffer_size{3 * 2000000};

        std::string result_path{"~/"};
        std::string seq_file{"~/"};
        std::string seq_name{"~/"};

        DataConfiguration(const cv::FileStorage fsSettings)
        {

            fsSettings["result_path"] >> result_path;

            voxel_scale = fsSettings["voxel_size"];
            truncation_distance = fsSettings["truncated_size"];
            int voxel_x = fsSettings["voxel_x"];
            int voxel_y = fsSettings["voxel_y"];
            int voxel_z = fsSettings["voxel_z"];
            volume_size = make_int3(voxel_x, voxel_y, voxel_z);

            float init_x = fsSettings["init_x"];
            float init_y = fsSettings["init_y"];
            float init_z = fsSettings["init_z"];

            float init_pos_x = volume_size.x / 2 * voxel_scale - init_x;
            float init_pos_y = volume_size.y / 2 * voxel_scale - init_y;
            float init_pos_z = volume_size.z / 2 * voxel_scale - init_z;
            init_pos = make_float3(init_pos_x, init_pos_y, init_pos_z);

            fsSettings["name"] >> seq_name;
        }
    };

    namespace internal
    {

        struct LogConfig
        {

            // std::ofstream imu_rotation_pose_diff;
            // std::ofstream fusion_rotation_pose;
            // std::ofstream imu_rotation_pose;
            // std::ofstream log_gravity;
            // std::ofstream log_translation;

            LogConfig(std::string dir_path)
            {
                //     imu_rotation_pose_diff= std::ofstream(dir_path+"imu_rotation_pose_diif.log");
                //     fusion_rotation_pose= std::ofstream(dir_path+"fusion_rotation_pose.log");
                //     imu_rotation_pose= std::ofstream(dir_path+"imu_rotation_pose.log");
                //     log_gravity= std::ofstream(dir_path+"imu_gravity.log");
                //     log_translation= std::ofstream(dir_path+"translation.log");

                // std::string dir_path="/home/jiazhao/code/rosefusion-imu/build/log/";
            }
        };

        struct FrameData
        {
            GpuMat depth_map;
            GpuMat color_map;
            GpuMat vertex_map;
            GpuMat normal_map;
            GpuMat shading_buffer;

            GpuMat gpu_block_count;
            cv::Mat block_count;

            GpuMat gpu_compact_vertex;
            cv::Mat compact_vertex;
            cv::Mat cpu_vertex_map;
            int compact_size = 0; // 有效点个数（其实是降采样之后的）

            explicit FrameData(const int image_height, const int image_width)
            {
                depth_map = cv::cuda::createContinuous(image_height, image_width, CV_32FC1);
                color_map = cv::cuda::createContinuous(image_height, image_width, CV_8UC3);
                vertex_map = cv::cuda::createContinuous(image_height, image_width, CV_32FC3);
                normal_map = cv::cuda::createContinuous(image_height, image_width, CV_32FC3);
                shading_buffer = cv::cuda::createContinuous(image_height, image_width, CV_8UC3);

                block_count = cv::Mat::zeros(1024, 1, CV_32SC1);
                compact_vertex = cv::Mat::zeros(image_height * image_width, 1, CV_32FC3);

                gpu_block_count = cv::cuda::createContinuous(1024, 1, CV_32SC1);
                gpu_compact_vertex = cv::cuda::createContinuous(image_height * image_width, 1, CV_32FC3);
                cpu_vertex_map = cv::Mat::zeros(image_height , image_width, CV_32FC3);
            }
        };

        struct VolumeData
        {
            GpuMat tsdf_volume;
            GpuMat weight_volume;
            GpuMat color_volume;
            int3 volume_size;
            float voxel_scale;

            VolumeData(const int3 _volume_size, const float _voxel_scale) : tsdf_volume(cv::cuda::createContinuous(_volume_size.y * _volume_size.z, _volume_size.x, CV_16SC1)),
                                                                            weight_volume(cv::cuda::createContinuous(_volume_size.y * _volume_size.z, _volume_size.x, CV_16SC1)),
                                                                            color_volume(cv::cuda::createContinuous(_volume_size.y * _volume_size.z, _volume_size.x, CV_8UC3)),
                                                                            volume_size(_volume_size), voxel_scale(_voxel_scale)
            {
                tsdf_volume.setTo(32767);
                weight_volume.setTo(0);
                color_volume.setTo(0);
            }
        };

        struct QuaternionData
        {
            std::vector<GpuMat> q;
            std::vector<cv::Mat> q_trans;
            int num = 20;

            QuaternionData(std::vector<int> particle_level, std::string PST_path) : q(60), q_trans(60)
            {

                for (int i = 0; i < num; i++)
                {
                    q_trans[i] = cv::Mat(particle_level[0], 6, CV_32FC1);
                    q[i] = cv::cuda::createContinuous(particle_level[0], 6, CV_32FC1);

                    q_trans[i] = cv::imread(PST_path + "pst_10240_" + std::to_string(i) + ".tiff", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
                    q_trans[i].ptr<float>(0)[0] = 0;
                    q_trans[i].ptr<float>(0)[1] = 0;
                    q_trans[i].ptr<float>(0)[2] = 0;
                    q_trans[i].ptr<float>(0)[3] = 0;
                    q_trans[i].ptr<float>(0)[4] = 0;
                    q_trans[i].ptr<float>(0)[5] = 0;
                    q[i].upload(q_trans[i]);
                }

                for (int i = num; i < num * 2; i++)
                {
                    q_trans[i] = cv::Mat(particle_level[1], 6, CV_32FC1);
                    q[i] = cv::cuda::createContinuous(particle_level[1], 6, CV_32FC1);

                    q_trans[i] = cv::imread(PST_path + "pst_3072_" + std::to_string(i - 20) + ".tiff", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
                    q_trans[i].ptr<float>(0)[0] = 0;
                    q_trans[i].ptr<float>(0)[1] = 0;
                    q_trans[i].ptr<float>(0)[2] = 0;
                    q_trans[i].ptr<float>(0)[3] = 0;
                    q_trans[i].ptr<float>(0)[4] = 0;
                    q_trans[i].ptr<float>(0)[5] = 0;
                    q[i].upload(q_trans[i]);
                }

                for (int i = num * 2; i < num * 3; i++)
                {
                    q_trans[i] = cv::Mat(particle_level[2], 6, CV_32FC1);
                    q[i] = cv::cuda::createContinuous(particle_level[2], 6, CV_32FC1);

                    q_trans[i] = cv::imread(PST_path + "pst_1024_" + std::to_string(i - 40) + ".tiff", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
                    q_trans[i].ptr<float>(0)[0] = 0;
                    q_trans[i].ptr<float>(0)[1] = 0;
                    q_trans[i].ptr<float>(0)[2] = 0;
                    q_trans[i].ptr<float>(0)[3] = 0;
                    q_trans[i].ptr<float>(0)[4] = 0;
                    q_trans[i].ptr<float>(0)[5] = 0;
                    q[i].upload(q_trans[i]);
                }
            }
        };

        struct SearchData
        {
            std::vector<GpuMat> gpu_search_count;
            std::vector<cv::Mat> search_count;
            std::vector<GpuMat> gpu_search_value;
            std::vector<cv::Mat> search_value;

            std::vector<GpuMat> gpu_imu_search_value;
            std::vector<cv::Mat> imu_search_value;

            std::vector<cv::Mat> imu_mat; // 这个暂时还不知道什么用处，5 X[100*7]，用于保存帧间imu数据的
            std::vector<GpuMat> gpu_imu_mat;

            cv::Mat imu_info; //用于估计重力的
            GpuMat gpu_imu_info;
            // cv::Mat imu_mat;
            // GpuMat gpu_imu_mat;

            SearchData(std::vector<int> particle_level) : // 为每一个粒子层次搜索需要进行空间分配
                                                          gpu_search_count(3), search_count(3), gpu_search_value(3), search_value(3), imu_mat(5), gpu_imu_mat(5),
                                                          gpu_imu_search_value(3), imu_search_value(3)
            {

                search_count[0] = cv::Mat::zeros(particle_level[0], 1, CV_32FC1);
                search_count[1] = cv::Mat::zeros(particle_level[1], 1, CV_32FC1);
                search_count[2] = cv::Mat::zeros(particle_level[2], 1, CV_32FC1);

                gpu_search_count[0] = cv::cuda::createContinuous(particle_level[0], 1, CV_32FC1);
                gpu_search_count[1] = cv::cuda::createContinuous(particle_level[1], 1, CV_32FC1);
                gpu_search_count[2] = cv::cuda::createContinuous(particle_level[2], 1, CV_32FC1);

                search_value[0] = cv::Mat::zeros(particle_level[0], 1, CV_32FC1);
                search_value[1] = cv::Mat::zeros(particle_level[1], 1, CV_32FC1);
                search_value[2] = cv::Mat::zeros(particle_level[2], 1, CV_32FC1);

                gpu_search_value[0] = cv::cuda::createContinuous(particle_level[0], 1, CV_32FC1);
                gpu_search_value[1] = cv::cuda::createContinuous(particle_level[1], 1, CV_32FC1);
                gpu_search_value[2] = cv::cuda::createContinuous(particle_level[2], 1, CV_32FC1);

                imu_search_value[0] = cv::Mat::zeros(particle_level[0], 1, CV_32FC1);
                imu_search_value[1] = cv::Mat::zeros(particle_level[1], 1, CV_32FC1);
                imu_search_value[2] = cv::Mat::zeros(particle_level[2], 1, CV_32FC1);

                gpu_imu_search_value[0] = cv::cuda::createContinuous(particle_level[0], 1, CV_32FC1);
                gpu_imu_search_value[1] = cv::cuda::createContinuous(particle_level[1], 1, CV_32FC1);
                gpu_imu_search_value[2] = cv::cuda::createContinuous(particle_level[2], 1, CV_32FC1);

                imu_mat[0] = cv::Mat::zeros(100, 7, CV_32FC1);
                imu_mat[1] = cv::Mat::zeros(100, 7, CV_32FC1);
                imu_mat[2] = cv::Mat::zeros(100, 7, CV_32FC1);
                imu_mat[3] = cv::Mat::zeros(100, 7, CV_32FC1);
                imu_mat[4] = cv::Mat::zeros(100, 7, CV_32FC1);

                gpu_imu_mat[0] = cv::cuda::createContinuous(100, 7, CV_32FC1);
                gpu_imu_mat[1] = cv::cuda::createContinuous(100, 7, CV_32FC1);
                gpu_imu_mat[2] = cv::cuda::createContinuous(100, 7, CV_32FC1);
                gpu_imu_mat[3] = cv::cuda::createContinuous(100, 7, CV_32FC1);
                gpu_imu_mat[4] = cv::cuda::createContinuous(100, 7, CV_32FC1);

                imu_info = cv::Mat::zeros(20, 3, CV_32FC1);
                gpu_imu_info = cv::cuda::createContinuous(20, 3, CV_32FC1);
            }
        };

        struct CloudData
        {
            GpuMat vertices;
            GpuMat normals;
            GpuMat color;

            cv::Mat host_vertices;
            cv::Mat host_normals;
            cv::Mat host_color;

            int *point_num;
            int host_point_num;

            explicit CloudData(const int max_number) : vertices{cv::cuda::createContinuous(1, max_number, CV_32FC3)},
                                                       normals{cv::cuda::createContinuous(1, max_number, CV_32FC3)},
                                                       color{cv::cuda::createContinuous(1, max_number, CV_8UC3)},
                                                       host_vertices{}, host_normals{}, host_color{}, point_num{nullptr}, host_point_num{}
            {
                vertices.setTo(0.f);
                normals.setTo(0.f);
                color.setTo(0.f);

                cudaMalloc(&point_num, sizeof(int));
                cudaMemset(point_num, 0, sizeof(int));
            }

            CloudData(const CloudData &) = delete;
            CloudData &operator=(const CloudData &data) = delete;

            void download()
            {
                vertices.download(host_vertices);
                normals.download(host_normals);
                color.download(host_color);

                cudaMemcpy(&host_point_num, point_num, sizeof(int), cudaMemcpyDeviceToHost);
            }
        };

    }
}

#endif
