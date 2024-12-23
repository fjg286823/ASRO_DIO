#include <rosefusion.h>
#include <iostream>
#include <fstream>

using cv::cuda::GpuMat;
using Matf61da = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;

namespace rosefusion {

    Pipeline::Pipeline(const CameraParameters _camera_config,
                       const DataConfiguration _data_config,
                       const ControllerConfiguration _controller_config,
                       const IMU_Model _imu_model) :
            camera_parameters(_camera_config), data_config(_data_config),
            controller_config(_controller_config),
            imu_model(_imu_model),
            volume(data_config.volume_size, data_config.voxel_scale),
            frame_data(_camera_config.image_height,_camera_config.image_width),
            particle_level{10240,3072,1024},PST(particle_level,_controller_config.PST_path),search_data(particle_level),
            current_pose{}, poses{}, frame_id{0}, last_model_frame{},iter_tsdf{_controller_config.init_fitness},rosefusion_log("/home/jiazhao/code/rosefusion-imu/build/log/")
    {
        current_pose.setIdentity();
        current_pose(0, 3) = data_config.init_pos.x;
        current_pose(1, 3) = data_config.init_pos.y;
        current_pose(2, 3) = data_config.init_pos.z;
        std::cout<<"voxel size: "<<data_config.voxel_scale<<std::endl;
        //voxel hash
        scene = new DWIO::ITMScene(true,data_config.voxel_scale,MEMORYDEVICE_CUDA);
        swapEngine = new DWIO::ITMSwappingEngine_CUDA();
        sceneRecoEngine = new DWIO::ITMSceneReconstructionEngine_CUDA();
        renderState_vh = new DWIO::ITMRenderState_VH(scene->index.noTotalEntries, MEMORYDEVICE_CUDA);
        sceneRecoEngine->ResetScene(scene);
        meshingEngine = new DWIO::ITMMeshingEngine_CUDA();
        // mesh = new DWIO::ITMMesh(MEMORYDEVICE_CUDA);

        camera_intrinsic(0, 0) = camera_parameters.focal_x;
        camera_intrinsic(1, 0) = camera_parameters.focal_y;
        camera_intrinsic(2, 0) = camera_parameters.principal_x;
        camera_intrinsic(3, 0) = camera_parameters.principal_y;
        color_intrinic = camera_intrinsic;
    }

    bool Pipeline::process_frame(const cv::Mat_<float>& depth_map, const cv::Mat_<cv::Vec3b>& color_map, cv::Mat& shaded_img)
    {

        printf("measurement\n");
        internal::surface_measurement(color_map, depth_map, frame_data, camera_parameters, data_config.depth_cutoff_distance);                                              
        printf("pose estimation\n");

        bool tracking_success { true };
        bool gravity_success {true};

        // for (int j=0; j<imu_model.v_iframes.size();j++){
        //     int n_imu=imu_model.v_iframes[j].v_imu.size();
        //     printf("imu_size: %d \n",n_imu);
        // }
        // ros::Rate rate(30.);


        if (frame_id > 0) { 
            printf("frame id: %d \n",frame_id);
            // initialization the roation and translation
            Eigen::Matrix4d init_relative_motion=Eigen::Matrix4d::Identity();
            // std::cout<<"back of poses:"<<poses.back()<<std::endl;
            if (frame_id<2){
                init_relative_motion=imu_model.integration_relative_rotation(poses.back()); //把上一帧位姿送给积分器（其实根本没用到！）。得到两帧之间的相对旋转
            }else{
                // init_relative_motion=imu_model.integration_relative_motion(poses.back());
                init_relative_motion=imu_model.integration_relative_rotation(poses.back());
            }





            std::cout<<"imu_model_size:"<< imu_model.v_imu.size()<<std::endl;
            //compute new pose
            std::cout << "init_relative_motion: \n" << init_relative_motion << std::endl;
            tracking_success = internal::pose_estimation(volume,PST,search_data,current_pose, frame_data, //采样位姿和偏差得到最优解！
            camera_parameters,controller_config,particle_level,&iter_tsdf,&previous_frame_success,
            initialize_search_size,init_relative_motion,rosefusion_log,imu_model);

            // std::cout<<"imu_model_size:"<< imu_model.v_imu.size()<<std::endl;
            
            //compute gravity
            gravity_success = internal::gravity_estimation(PST,search_data,current_pose,poses.back(),
            particle_level,initialize_search_size,rosefusion_log,imu_model);//采样G对应的旋转矩阵和V来修正G,同时采样偏差进一步优化更新偏差,残差都是积分的imu结果和上一步粒子滤波优化的结果

            // imu_model.update_frame();
            // if (frame_id==11){
            //     exit(0);
            // }

            
        }

        //解决跟踪失败问题，需要把所有的pose都保存而不是只有成功才保存！
        poses.push_back(current_pose);

        if (!tracking_success )
            return false;

        // poses.push_back(current_pose); //原始实现！
        printf("surface reconsturction\n");

        internal::cuda::surface_reconstruction(frame_data.depth_map, frame_data.color_map,
                                               volume, camera_parameters, data_config.truncation_distance,
                                               current_pose.inverse());

        printf("render surface\n");

        if (controller_config.render_surface){
            internal::cuda::surface_prediction(volume,
                                            frame_data.shading_buffer,
                                            camera_parameters, data_config.truncation_distance,
                                            data_config.init_pos,
                                            shaded_img,
                                            current_pose);
        }

        frame_id+=1;
        return true;
    }

    void Pipeline::add_imu(imu_data id)
    {
        imu_model.v_imu.push_back(id);
    }

    // void Pipleline:sample_imu()
    // {
    //     if (imu_model.v_imu.size()>15){

    //     }
    // }

    // void Pipeline::add_imu_initial(imu_data id,double time)
    // {
    //     imu_data id_frame;
    //     id_frame.time = time;
    //     id_frame.rx   = id.rx;
    //     id_frame.ry   = id.ry;
    //     id_frame.rz   = id.rz;
    //     id_frame.dx   = id.dx;
    //     id_frame.dy   = id.dy;
    //     id_frame.dz   = id.dz;
    //     imu_model.v_imu.push_back(id_frame);
    // }

    // void Pipeline::add_imu_end(double time)
    // {   
    //     imu_data id_last=imu_model.v_imu.back();
    //     imu_data id_frame;
    //     id_frame.time = time;
    //     id_frame.rx   = id_last.rx;
    //     id_frame.ry   = id_last.ry;
    //     id_frame.rz   = id_last.rz;
    //     id_frame.dx   = id_last.dx;
    //     id_frame.dy   = id_last.dy;
    //     id_frame.dz   = id_last.dz;      

    //     imu_model.v_imu.push_back(id_frame);
    // }





    void Pipeline::set_timegap(double time_gap){
        imu_model.current_time_gap=time_gap;

    }

    void Pipeline::get_poses(std::vector<std::string> v_index) 
    {
        Eigen::Matrix4d init_pose=poses[0];
        std::ofstream trajectory;
        trajectory.open(data_config.result_path+data_config.seq_name+".txt");
        std::cout<<data_config.result_path+data_config.seq_name+".txt"<<std::endl;
        int iter_count=0;
        for (auto pose : poses){
            Eigen::Matrix4d temp_pose=init_pose.inverse()*pose;
            Eigen::Matrix3d rotation_m=temp_pose.block(0,0,3,3);
            Eigen::Vector3d translation=temp_pose.block(0,3,3,1)/1000;
            Eigen::Quaterniond q(rotation_m);
            trajectory<<v_index[iter_count]<<" "<<translation.x()<<" "<<translation.y()<<" "<<translation.z()<<\
            " "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
            iter_count++;
        }
        trajectory.close();
    }

    Eigen::Matrix4d Pipeline::return_pose()
    {

        return poses[0].inverse()*poses[poses.size()-1];
    }


    PointCloud Pipeline::extract_pointcloud() const
    {
        PointCloud cloud_data = internal::cuda::extract_points(volume, data_config.pointcloud_buffer_size);
        return cloud_data;
    }

    void Pipeline::get_submap_size(int* submap_size,cv::Mat& vertex_map,float voxel_scale,float block_size) {
        int x_min = INT_MAX;
        int y_min = INT_MAX;
        int z_min = INT_MAX;
        int x_max = 0;
        int y_max = 0;
        int z_max = 0;

        for(int i=0;i<vertex_map.rows;i++) {
            for(int j=0;j<vertex_map.cols;j++) {
                cv::Vec3f point = vertex_map.at<cv::Vec3f>(i,j);
                Eigen::Vector4f point_eigen;
                point_eigen[0] = point[0];
                point_eigen[1] = point[1];
                point_eigen[2] = point[2];
                point_eigen[3] = 1.0;
                Eigen::Vector4f point_world = current_pose.cast<float>()*point_eigen;
                x_max = point_world[0] >x_max?point_world[0]: x_max;
                y_max = point_world[1] >y_max?point_world[1]:y_max;
                z_max = point_world[2] >z_max?point_world[2]:z_max;
                x_min = point_world[0] <x_min?point_world[0]:x_min;
                y_min = point_world[1] <y_min?point_world[1]:y_min;
                z_min = point_world[2] <z_min?point_world[2]:z_min;
            }
        }

        submap_size[0] = std::ceil(x_min/voxel_scale/block_size) -20;
        submap_size[1] = std::ceil(y_min/voxel_scale/block_size) -20;
        submap_size[2] = std::ceil(z_min/voxel_scale/block_size) -20;
        submap_size[3] = std::ceil(x_max/voxel_scale/block_size) +20;
        submap_size[4] = std::ceil(y_max/voxel_scale/block_size) +20;
        submap_size[5] = std::ceil(z_max/voxel_scale/block_size) +20;
    }


    bool Pipeline::process_frame_hash(const cv::Mat_<float>& depth_map, const cv::Mat_<cv::Vec3b>& color_map, cv::Mat& shaded_img) {
        // printf("measurement\n");
        internal::surface_measurement(color_map, depth_map, frame_data, camera_parameters, data_config.depth_cutoff_distance);


        bool tracking_success { true };
        bool gravity_success {true};

        int submap_size[] = {-100, -100, -100, 100, 100, 100};
        // get_submap_size(submap_size,frame_data.cpu_vertex_map,volume.voxel_scale,SDF_BLOCK_SIZE); //这里需要考虑当前的位姿转到世界坐标系下！
        std::cout<<"submap size： "<<submap_size[0]<<" "<<submap_size[1]<<"  "<<submap_size[2]<<" "<<submap_size[3]<<" "<<submap_size[4]<<" "<<submap_size[5]<<std::endl;

        sceneRecoEngine->AllocateScene(scene, //会把现有的block（在子图里的）设置为可见，这样就会被转入到gpu中
                               frame_data.depth_map,
                               current_pose,
                               renderState_vh,
                               camera_intrinsic,
                               data_config.truncation_distance,
                               submap_size,
                               false,
                               false);

        swapEngine->IntegrateGlobalIntoLocal(scene, renderState_vh, false);


        printf("pose estimation\n");
        if (frame_id > 0) {
            // printf("frame id: %d \n",frame_id);
            // initialization the roation and translation
            Eigen::Matrix4d init_relative_motion=Eigen::Matrix4d::Identity();
            // std::cout<<"back of poses:"<<poses.back()<<std::endl;
            if (frame_id<2){
                init_relative_motion=imu_model.integration_relative_rotation(poses.back()); //把上一帧位姿送给积分器（其实根本没用到！）。得到两帧之间的相对旋转
            }else{
                // init_relative_motion=imu_model.integration_relative_motion(poses.back());
                init_relative_motion=imu_model.integration_relative_rotation(poses.back());
            }

            std::cout<<"imu_model_size:"<< imu_model.v_imu.size()<<std::endl;
            //compute new pose
            std::cout << "init_relative_motion: \n" << init_relative_motion << std::endl;
            tracking_success = internal::pose_estimation_hash(scene->localVBA.GetVoxelBlocks(),scene->index.GetEntries(),volume,PST,search_data,current_pose, frame_data, //采样位姿和偏差得到最优解！
                                                                camera_parameters,controller_config,particle_level,&iter_tsdf,&previous_frame_success,
                                                                initialize_search_size,init_relative_motion,rosefusion_log,imu_model);


            gravity_success = internal::gravity_estimation(PST,search_data,current_pose,poses.back(),
                                                        particle_level,initialize_search_size,rosefusion_log,imu_model);//采样G对应的旋转矩阵和V来修正G,同时采样偏差进一步优化更新偏差,残差都是积分的imu结果和上一步粒子滤波优化的结果
        }

        poses.push_back(current_pose);


        if (!tracking_success )
            return false;

        sceneRecoEngine->AllocateSceneFromDepth(scene,
                                        frame_data.depth_map,
                                        current_pose,
                                        renderState_vh,
                                        camera_intrinsic,
                                        data_config.truncation_distance, submap_size);
        sceneRecoEngine->IntegrateIntoScene(scene,
                                    frame_data.depth_map,
                                    frame_data.color_map,
                                    current_pose.inverse(),
                                    renderState_vh,
                                    camera_intrinsic,
                                    color_intrinic,
                                    camera_parameters.extrinic_d_2_c,
                                    data_config.truncation_distance);

        swapEngine->IntegrateGlobalIntoLocal(scene, renderState_vh, true);

        printf("render surface\n");

        if (controller_config.render_surface){
            internal::cuda::surface_prediction_hash(scene,volume,
                                            frame_data.shading_buffer,
                                            camera_parameters, data_config.truncation_distance,
                                            data_config.init_pos,
                                            shaded_img,
                                            current_pose);
        }
        sceneRecoEngine->showHashTableAndVoxelAllocCondition(scene,renderState_vh);

        frame_id+=1;
        return true;
    }




    void export_ply(const std::string& filename, const PointCloud& point_cloud)
    {
        std::ofstream file_out { filename };
        if (!file_out.is_open())
            return;

        file_out << "ply" << std::endl;
        file_out << "format ascii 1.0" << std::endl;
        file_out << "element vertex " << point_cloud.num_points << std::endl;
        file_out << "property float x" << std::endl;
        file_out << "property float y" << std::endl;
        file_out << "property float z" << std::endl;
        file_out << "property float nx" << std::endl;
        file_out << "property float ny" << std::endl;
        file_out << "property float nz" << std::endl;
        file_out << "property uchar red" << std::endl;
        file_out << "property uchar green" << std::endl;
        file_out << "property uchar blue" << std::endl;
        file_out << "end_header" << std::endl;

        for (int i = 0; i < point_cloud.num_points; ++i) {
            float3 vertex = point_cloud.vertices.ptr<float3>(0)[i];
            float3 normal = point_cloud.normals.ptr<float3>(0)[i];
            uchar3 color = point_cloud.color.ptr<uchar3>(0)[i];
            file_out << vertex.x << " " << vertex.y << " " << vertex.z << " " << normal.x << " " << normal.y << " "
                     << normal.z << " ";
            file_out << static_cast<int>(color.x) << " " << static_cast<int>(color.y) << " "
                     << static_cast<int>(color.z) << std::endl;
        }
    }




}