#ifndef ROSEFUSION_H
#define ROSEFUSION_H

#include "data_types.h"

#include "ITMScene.h"
#include "ITMMesh.h"
#include "ITMSwappingEngine_CUDA.h"
#include "ITMSceneReconstructionEngine_CUDA.h"
#include "ITMMeshingEngine_CUDA.h"
using Matf61da = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;

namespace rosefusion {

    class Pipeline {

    public:

        // Pipeline();

        Pipeline(const CameraParameters _camera_config,
                 const DataConfiguration _data_config,
                 const ControllerConfiguration _controller_config,
                 const IMU_Model _imu_model);

        ~Pipeline() = default;


            bool process_frame(const cv::Mat_<float>& depth_map, const cv::Mat_<cv::Vec3b>& color_map, cv::Mat& shaded_img);

            bool process_frame_hash(const cv::Mat_<float>& depth_map, const cv::Mat_<cv::Vec3b>& color_map, cv::Mat& shaded_img);

            void get_poses(std::vector<std::string> v_index) ;

            PointCloud extract_pointcloud() const;

            void update_imu(Eigen::Vector3d avg_measure_acc,double time_gap);

            void add_imu(imu_data id);
            void add_imu_initial(imu_data id,double time);
            void add_imu_end( double time);
            Eigen::Matrix4d return_pose();

            void set_timegap(double time_gap);

            void get_submap_size(int* submap_size,cv::Mat& vertex_map,float voxel_scale,float block_size);

            const CameraParameters camera_parameters;
            const ControllerConfiguration controller_config;

            Vector4f camera_intrinsic;
            Vector4f color_intrinic;

        private:
            const DataConfiguration data_config;
            const std::vector<int> particle_level;


            internal::LogConfig rosefusion_log;

            IMU_Model imu_model;

            float iter_tsdf;
            internal::VolumeData volume;
            internal::QuaternionData PST; 
            internal::SearchData search_data;
            internal::FrameData frame_data;
            Eigen::Matrix4d current_pose;
            std::vector<Eigen::Matrix4d> poses;
            bool previous_frame_success=false;
            Matf61da initialize_search_size;
            int frame_id;
            cv::Mat last_model_frame;

            //voxel hash
            DWIO::ITMScene *scene;
            DWIO::ITMMesh *mesh;
            DWIO::ITMSwappingEngine_CUDA *swapEngine;
            DWIO::ITMRenderState_VH *renderState_vh;
            DWIO::ITMSceneReconstructionEngine_CUDA *sceneRecoEngine;
            DWIO::ITMMeshingEngine_CUDA *meshingEngine;

    };

    void export_ply(const std::string& filename, const PointCloud& point_cloud);



    namespace internal {


        void surface_measurement(const cv::Mat_<cv::Vec3b>& color_map,
                                      const cv::Mat_<float>& depth_map,
                                      FrameData& frame_data,
                                      const CameraParameters& camera_params,
                                      const float depth_cutoff);




        bool pose_estimation(const VolumeData& volume,
                             const QuaternionData& quaternions,
                              SearchData& search_data,
                             Eigen::Matrix4d& pose,
                             FrameData& frame_data,
                             const CameraParameters& cam_params,
                             const ControllerConfiguration& controller_config,
                             const std::vector<int> particle_level,
                             float * iter_tsdf,
                             bool * previous_frame_success,
                             Matf61da& initialize_search_size,
                             const Eigen::Matrix4d& imu_relative_q,
                             LogConfig& log_config,
                             IMU_Model& imu_model
                            );

        bool pose_estimation_hash(const ITMVoxel_d *voxelData,
                    const ITMHashEntry *hashTable,const VolumeData& volume,
                     const QuaternionData& quaternions,
                      SearchData& search_data,
                     Eigen::Matrix4d& pose,
                     FrameData& frame_data,
                     const CameraParameters& cam_params,
                     const ControllerConfiguration& controller_config,
                     const std::vector<int> particle_level,
                     float * iter_tsdf,
                     bool * previous_frame_success,
                     Matf61da& initialize_search_size,
                     const Eigen::Matrix4d& imu_relative_motion,
                     LogConfig& log,
                     IMU_Model& imu_model);

        bool gravity_estimation(const QuaternionData& quaternions,
                              SearchData& search_data,
                              const Eigen::Matrix4d& current_pose,
                              const Eigen::Matrix4d& last_pose,
                              const std::vector<int> particle_level,
                              Matf61da& initialize_search_size,
                              LogConfig& log,
                              IMU_Model& imu_model);





        namespace cuda {


            void surface_reconstruction(const cv::cuda::GpuMat& depth_image,
                                        const cv::cuda::GpuMat& color_image,
                                        VolumeData& volume,
                                        const CameraParameters& cam_params,
                                        const float truncation_distance,
                                        const Eigen::Matrix4d& model_view);


            void surface_prediction(const VolumeData& volume,
                                    cv::cuda::GpuMat& shading_buffer,
                                    const CameraParameters& cam_parameters,
                                    const float truncation_distance,
                                    const float3 init_pos,
                                    cv::Mat& shaded_img,
                                    const Eigen::Matrix4d& pose);
            void surface_prediction_hash(DWIO::ITMScene *scene,const VolumeData& volume,
                                        GpuMat& shading_buffer,
                                        const CameraParameters& cam_parameters,
                                        const float truncation_distance,
                                        const float3 init_pos,
                                        cv::Mat& shading_img,
                                        const Eigen::Matrix4d& pose);

            PointCloud extract_points(const VolumeData& volume, const int buffer_size);

        }

    }
}
#endif 
