#include <rosefusion.h>

using Matf31da = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matf61da = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Matrix3frm = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;


namespace rosefusion {
    namespace internal {

        namespace cuda { 



            bool particle_evaluation(const VolumeData& volume,
                                const QuaternionData& quaterinons, SearchData& search_data ,
                                const Eigen::Matrix3d& rotation_current, const Matf31da& translation_current,
                                const cv::cuda::GpuMat& gpu_compact_vertex, int compact_size,

                                const Eigen::Matrix3d& rotation_previous_inv, const Matf31da& translation_previous,
                                const CameraParameters& cam_params, const int particle_index,const int particle_size,
                                const Matf61da& search_size, int resolution_level,const int level_index,
                                Eigen::Matrix<double, 7, 1>& mean_transform, float * tsdf, IMU_Model& imu_model,
                                const Eigen::Vector3d incre_translation, const Eigen::Matrix3d incre_rotation,
                                const Eigen::Matrix3d frame_rotation, int iter_times
                                );
            bool particle_evaluation_hash(const ITMVoxel_d *voxelData,const ITMHashEntry *hashTable,const VolumeData& volume,
                         const QuaternionData &quaterinons, SearchData &search_data, const Eigen::Matrix3d &rotation_current, const Matf31da &translation_current,
                         const cv::cuda::GpuMat &gpu_compact_vertex, int compact_size,
                         const Eigen::Matrix3d &rotation_previous_inv, const Matf31da &translation_previous,
                         const CameraParameters &cam_params, const int particle_index, const int particle_size,
                         const Matf61da &search_size, int level, const int level_index,
                         Eigen::Matrix<double, 7, 1> &mean_transform, float *min_tsdf, IMU_Model &imu_model,
                         const Eigen::Vector3d incre_translation, const Eigen::Matrix3d incre_rotation,
                         const Eigen::Matrix3d frame_rotation, int iter_times);
        }

 
        void update_seach_size(const float tsdf, const float scaling_coefficient,Matf61da& search_size, Eigen::Matrix<double, 7, 1>& mean_transform )
        {
            
            double s_tx=fabs(mean_transform(0,0))+1e-3;
            double s_ty=fabs(mean_transform(1,0))+1e-3;
            double s_tz=fabs(mean_transform(2,0))+1e-3; 

            double s_qx=fabs(mean_transform(4,0))+1e-3; 
            double s_qy=fabs(mean_transform(5,0))+1e-3;
            double s_qz=fabs(mean_transform(6,0))+1e-3;

            double trans_norm=sqrt(s_tx*s_tx+s_ty*s_ty+s_tz*s_tz+s_qx*s_qx+s_qy*s_qy+s_qz*s_qz);


            double normal_tx=s_tx/trans_norm;
            double normal_ty=s_ty/trans_norm;
            double normal_tz=s_tz/trans_norm;
            double normal_qx=s_qx/trans_norm;
            double normal_qy=s_qy/trans_norm;
            double normal_qz=s_qz/trans_norm;

            search_size(3,0) = scaling_coefficient * tsdf*normal_qx+1e-3;
            search_size(4,0) = scaling_coefficient * tsdf*normal_qy+1e-3;
            search_size(5,0) = scaling_coefficient * tsdf*normal_qz+1e-3;  
            search_size(0,0) = scaling_coefficient * tsdf*normal_tx+1e-3;
            search_size(1,0) = scaling_coefficient * tsdf*normal_ty+1e-3;
            search_size(2,0) = scaling_coefficient * tsdf*normal_tz+1e-3;
        }


        Eigen::Quaterniond rotation_2_quaternion(Eigen::Matrix3d rotation){
            
            double tr=rotation(0,0)+rotation(1,1)+rotation(2,2);
            double qw;
            double qx;
            double qy;
            double qz;

            double m00=rotation(0,0);
            double m01=rotation(0,1);
            double m02=rotation(0,2);
            double m10=rotation(1,0);
            double m11=rotation(1,1);
            double m12=rotation(1,2);
            double m20=rotation(2,0);
            double m21=rotation(2,1);
            double m22=rotation(2,2);


            if (tr > 0) { 
                float S = sqrt(tr+1.0) * 2; // S=4*qw 
                qw = 0.25 * S;
                qx = (m21 - m12) / S;
                qy = (m02 - m20) / S; 
                qz = (m10 - m01) / S; 
            } else if ((m00 > m11)&(m00 > m22)) { 
                float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
                qw = (m21 - m12) / S;
                qx = 0.25 * S;
                qy = (m01 + m10) / S; 
                qz = (m02 + m20) / S; 
            } else if (m11 > m22) { 
                float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
                qw = (m02 - m20) / S;
                qx = (m01 + m10) / S; 
                qy = 0.25 * S;
                qz = (m12 + m21) / S; 
            } else { 
                float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
                qw = (m10 - m01) / S;
                qx = (m02 + m20) / S;
                qy = (m12 + m21) / S;
                qz = 0.25 * S;
            }
            return Eigen::Quaterniond(qw,qx,qy,qz);

        }

        static bool previous_success=true;

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
                             const Eigen::Matrix4d& imu_relative_motion,
                             LogConfig& log,
                             IMU_Model& imu_model)
        {   
            

            bool use_translation = true;
            bool use_rotation = true;

            Eigen::Matrix3d current_global_rotation = pose.block(0, 0, 3, 3);
            Eigen::Vector3d current_global_translation = pose.block(0, 3, 3, 1);


            Eigen::Matrix3d frame_global_rotation= pose.block(0,0,3,3);

            Eigen::Matrix3d init_camera_rotation_incremental=imu_relative_motion.block(0, 0, 3, 3);
            Eigen::Vector3d init_camera_translation_incremental=imu_relative_motion.block(0, 3, 3, 1);
            Eigen::Quaterniond imu_relative_q(init_camera_rotation_incremental);
            Eigen::Matrix3d record_camera_rotation_incremental;
            record_camera_rotation_incremental<<1,0,0,0,1,0,0,0,1;
            Eigen::Vector3d record_camera_trans_incremental(0,0,0);



            Matf61da search_size;

            float lens= controller_config.scaling_coefficient1*(*iter_tsdf); //0.25*0.6初始值，之后就是上一帧对应的残差！这会导致lens也变成0这样就不搜索了，一直是不动的位姿
            if(!previous_success){
                lens = 0.25 * 0.6;
                lens = lens * 1.2;
            }
            search_size<< lens, lens, lens, lens, lens, lens;
            // std::cout << "lens: " << lens << std::endl;

            //update initialize pose


            // Eigen::Matrix3d init_camera_rotation_incremental;


            std::cout<<"============ Pose estimation ==============\n";

            // std::cout<<"imu relative q: "<<imu_relative_q.w()<<" "<<imu_relative_q.x()<<" "<<
            // imu_relative_q.y()<<" "<<imu_relative_q.z()<<std::endl;

            // std::cout<<"imu trm q: "<<trm.imu_residual_rotation.x()<<" "<<
            // trm.imu_residual_rotation.y()<<" "<<trm.imu_residual_rotation.z()<<std::endl;

            // qx=imu_relative_q.x()+trm.imu_residual_rotation.x();
            // qy=imu_rel ative_q.y()+trm.imu_residual_rotation.y();
            // qz=imu_relative_q.z()+trm.imu_residual_rotation.z();


            double qx;
            double qy;
            double qz;
            double qw;

            // double tx=init_camera_translation_incremental.x();
            // double ty=init_camera_translation_incremental.y();
            // double tz=init_camera_translation_incremental.z();




            // qx= fabs(qx) + 1e-4;
            // qy= fabs(qy) + 1e-4;
            // qz= fabs(qz) + 1e-4;


            // tx= fabs(tx) + 1e-3;
            // ty= fabs(ty) + 1e-3;
            // tz= fabs(tz) + 1e-3;




            // //update search size
            // double q_norm=sqrt(qx*qx+qy*qy+qz*qz);
            // double t_norm=sqrt(tx*tx+ty*ty+tz*tz);

            // if (use_rotation){
            //     search_size(3,0)=lens*qx/q_norm/2;
            //     search_size(4,0)=lens*qy/q_norm/2;
            //     search_size(5,0)=lens*qz/q_norm/2;
            //     // search_size(3,0)=qx;
            //     // search_size(4,0)=qy;
            //     // search_size(5,0)=qz;


            // }else{
            //     search_size(3,0)=lens;
            //     search_size(4,0)=lens;
            //     search_size(5,0)=lens;
            // }

            // if (use_translation){
            //     // search_size(0,0)= tx;
            //     // search_size(1,0)= ty;
            //     // search_size(2,0)= tz;
            //     search_size(0,0)=lens*tx/t_norm*1.7;
            //     search_size(1,0)=lens*ty/t_norm*1.7;
            //     search_size(2,0)=lens*tz/t_norm*1.7; 
            //     // search_size(0,0)=lens;
            //     // search_size(1,0)=lens;
            //     // search_size(2,0)=lens;

            // }else{
            //     search_size(0,0)=lens;
            //     search_size(1,0)=lens;
            //     search_size(2,0)=lens;
            // }

            // search_size(0,0)=t_norm*2;
            // search_size(1,0)=t_norm*2;
            // search_size(2,0)=t_norm*2;





            //用于R^T*(P-T)转换到相机坐标系下！
            Eigen::Matrix3d previous_global_rotation_inverse(current_global_rotation.inverse());
            Eigen::Vector3d previous_global_translation =  current_global_translation;

            //得到预测值
            current_global_rotation = init_camera_rotation_incremental * current_global_rotation;
            if(use_translation){
                current_global_translation= init_camera_translation_incremental*1000 + current_global_translation;
            }else{
                current_global_translation=  current_global_translation;
            }


            if(use_rotation){
                record_camera_rotation_incremental=init_camera_rotation_incremental;
            }
            std::cout<<"record_camera_rotation_incremental"<<record_camera_rotation_incremental<<"\n";
            if (use_translation){
                record_camera_trans_incremental=init_camera_translation_incremental;
            }



  

            // std::cout<<"record_camera_rotation_incremental"<<record_camera_rotation_incremental<<std::endl;
            // exit(0);
            // int particle_index[20] ={0,1+20,2+40,3,4+20,5+40,6+0,7+20,8+40,
            //                         9+0,10+20,11+40,12+0,13+20,14+40,
            //                         15+0,16+20,17+40,18+0,19+20};
            // int level[20] = {160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640};
            int level[20] = {160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640};//这个level是用来干嘛的！在ROSEFusion里是用来跳点的！
            // int level[20] = {160,576,2560,160,576,2560,160,576,2560,160,576,2560,160,576,2560,160,576,2560,160,576};

            // int level[20] = {224,576,1280,224,576,1280,224,576,1280,224,576,1280,224,576,1280,224,576,1280,224,576};
            // int level[20] = {160,800,2560,160,800,2560,160,800,2560,160,800,2560,160,800,2560,160,800,2560,160,800};

            int particle_index[20] ={0,1+20,2+40,3,4+20,5+40,6+0,7+20,8+40,
                                    9+0,10+20,11+40,12+0,13+20,14+40,
                                    15+0,16+20,17+40,18+0,19+20}; //用来对应不同的粒子数目

            // int level[20] = {192,640,192,640,192,640,192,640,192,640,192,640,192,640,192,640,192,640,192,640};
            // int particle_index[20] ={0,20,1,21,2,22,3,23,4,24,5,25,6,26,7,27,8,28,9,29};


            // int level[21];

            // for (int i=0; i<7; i++){
            //     level[(i*3+0)%20]=(frame_data.compact_size-159)/160;
            //     level[(i*3+1)%20]=(frame_data.compact_size-639)/640;
            //     level[(i*3+2)%20]=(frame_data.compact_size-2559)/2560;
            // }



            int count_particle=0;//对应particle_index用于表明使用哪种粒子来搜索
            int level_index=3;
            bool success=true;
            // bool previous_success=true;

            int count=0; //记录迭代数目
            int count_success=0;
            float min_tsdf;


            //init imu data
            for (int i=0; i<imu_model.v_imu.size(); i++){ 
                search_data.imu_mat[2].ptr<float>(i)[0]=imu_model.v_imu[i].time;
                search_data.imu_mat[2].ptr<float>(i)[1]=imu_model.v_imu[i].dx;
                search_data.imu_mat[2].ptr<float>(i)[2]=imu_model.v_imu[i].dy;
                search_data.imu_mat[2].ptr<float>(i)[3]=imu_model.v_imu[i].dz;
                search_data.imu_mat[2].ptr<float>(i)[4]=imu_model.v_imu[i].rx;
                search_data.imu_mat[2].ptr<float>(i)[5]=imu_model.v_imu[i].ry;
                search_data.imu_mat[2].ptr<float>(i)[6]=imu_model.v_imu[i].rz;
            }

            search_data.gpu_imu_mat[2].upload(search_data.imu_mat[2]);







            while(true){

                Eigen::Matrix<double, 7, 1> mean_transform=Eigen::Matrix<double, 7, 1>::Zero();
  
                if(count==controller_config.max_iteration){
                    break;
                }

                if (!success){//如果此次搜索不成功就重新从最开始的粒子群开始搜索！
                    count_particle=0;
                }
                //执行一次粒子滤波，得到新的位姿和imu零偏！
                success=cuda::particle_evaluation(volume,quaternions,search_data,current_global_rotation, current_global_translation,
                                    frame_data.gpu_compact_vertex, frame_data.compact_size,
                                    previous_global_rotation_inverse, previous_global_translation,
                                    cam_params,particle_index[count_particle],particle_level[particle_index[count_particle]/20], //particle_level[particle_index[count_particle]/20]对应三种粒子群中的一个，表明该粒子群个数
                                    search_size,level[count_particle],level_index,
                                    mean_transform,&min_tsdf,imu_model,record_camera_trans_incremental,
                                    record_camera_rotation_incremental,frame_global_rotation,count);


                if (count==0 && !success)
                {
                    *iter_tsdf=min_tsdf;
                }
            
                qx=mean_transform(4,0);
                qy=mean_transform(5,0);
                qz=mean_transform(6,0);

                if (success){
                    if (count_particle<19){
                        ++count_particle;

                    }
                    ++count_success;

                    auto camera_translation_incremental = mean_transform.head<3>();
                    qw=mean_transform(3,0);
                    Eigen::Matrix3d camera_rotation_incremental;

                    camera_rotation_incremental << 1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw),
                                                    2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw),
                                                    2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy);

                    current_global_translation =  current_global_translation + camera_translation_incremental*1000;
                    current_global_rotation = camera_rotation_incremental * current_global_rotation;

                    //用于记录最后优化成功后的增量平移和旋转
                    record_camera_rotation_incremental =camera_rotation_incremental *record_camera_rotation_incremental;
                    record_camera_trans_incremental+=camera_translation_incremental;
                }
                

                level_index+=311;
                // static_cast<unsigned int>(std::ceil((float)particle_size /block.y/ block.x));
                int stride=static_cast<unsigned int>(std::ceil((frame_data.compact_size*1.f-level[count_particle]+1)/level[count_particle]));
                // std::cout<<"x1 stride:"<<stride<<std::endl;
                stride = stride<=0? 1: stride;
                // level_index=level_index%(level[count_particle]*level[count_particle]);
                level_index=level_index%stride;
                //更新搜索空间！
                update_seach_size(min_tsdf,controller_config.scaling_coefficient2,search_size,mean_transform);

                ++count;

            }

            if (count_success==0 ){
                previous_success = false;
                return false;
                
            }

            pose.block(0, 0, 3, 3) = current_global_rotation;
            pose.block(0, 3, 3, 1) = current_global_translation;
            std::cout<<"record incremental rotation: \n"<<record_camera_rotation_incremental<<std::endl;
            std::cout<<"=============record incremental translation: ==============\n"<<record_camera_trans_incremental<<std::endl;
            std::cout<<"error: \n"<<(record_camera_trans_incremental-init_camera_translation_incremental).norm()<<std::endl;


            std::cout<<"imu_model.bias_gyr:\n"<<imu_model.bias_gyr<<std::endl;
            std::cout<<"imu_model.bias_acc:\n"<<imu_model.bias_acc<<std::endl;


            // log.log_translation<< "init translation: \n"<<init_camera_translation_incremental<<std::endl;
            // log.log_translation<< "record translation: \n"<<record_camera_trans_incremental<<std::endl;

            // log.log_translation<< "init trans norm: \n"<<(init_camera_translation_incremental).norm()<<std::endl;
            // log.log_translation<< "record trans norm: \n"<<(record_camera_trans_incremental).norm()<<std::endl;

            // log.log_translation<< "error: \n"<<(record_camera_trans_incremental-init_camera_translation_incremental).norm()<<std::endl;
            // log.log_translation<<"=================================\n";


            // log.log_translation<<init_camera_translation_incremental.x()<<" "<<init_camera_translation_incremental.y()<<" "<<init_camera_translation_incremental.z()<<" ";
            // log.log_translation<<record_camera_trans_incremental.x()<<" "<<record_camera_trans_incremental.y()<<" "<<record_camera_trans_incremental.z()<<" ";
            // log.log_translation<<(init_camera_translation_incremental).norm()<<" "<<(record_camera_trans_incremental).norm()<<" "<<(record_camera_trans_incremental-init_camera_translation_incremental).norm()<<"\n";







            // Eigen::Quaterniond record_q=rotation_2_quaternion(record_camera_rotation_incremental);
            // printf("record q: %f %f %f %f\n",record_q.w(),record_q.x(),record_q.y(),record_q.z());

            // printf("diff q: %f %f %f %f\n",record_q.w()-imu_relative_q.w(),record_q.x()-imu_relative_q.x(),record_q.y()-imu_relative_q.y(),record_q.z()-imu_relative_q.z());

            // log.imu_rotation_pose_diff<<record_q.w()-imu_relative_q.w()
            // <<" "<< record_q.x()-imu_relative_q.x()
            // <<" " << record_q.y()-imu_relative_q.y()
            // <<" " << record_q.z()-imu_relative_q.z()<<"\n";

            // // trm.imu_residual_rotation<<record_q.x(),record_q.y(),record_q.z();

            // log.fusion_rotation_pose<<record_q.w()
            // <<" "<< record_q.x()
            // <<" " << record_q.y()
            // <<" " << record_q.z()<<"\n";

            // log.imu_rotation_pose<<imu_relative_q.w()
            // <<" "<< imu_relative_q.x()
            // <<" " << imu_relative_q.y()
            // <<" " << imu_relative_q.z()<<"\n";

            // imu_model.compute_tracking_velocity(record_camera_trans_incremental);

            previous_success = true;
            return true;
        }
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
                             IMU_Model& imu_model)
        {


            bool use_translation = true;
            bool use_rotation = true;

            Eigen::Matrix3d current_global_rotation = pose.block(0, 0, 3, 3);
            Eigen::Vector3d current_global_translation = pose.block(0, 3, 3, 1);


            Eigen::Matrix3d frame_global_rotation= pose.block(0,0,3,3);

            Eigen::Matrix3d init_camera_rotation_incremental=imu_relative_motion.block(0, 0, 3, 3);
            Eigen::Vector3d init_camera_translation_incremental=imu_relative_motion.block(0, 3, 3, 1);
            Eigen::Quaterniond imu_relative_q(init_camera_rotation_incremental);
            Eigen::Matrix3d record_camera_rotation_incremental;
            record_camera_rotation_incremental<<1,0,0,0,1,0,0,0,1;
            Eigen::Vector3d record_camera_trans_incremental(0,0,0);



            Matf61da search_size;

            float lens= controller_config.scaling_coefficient1*(*iter_tsdf); //0.25*0.6初始值，之后就是上一帧对应的残差！这会导致lens也变成0这样就不搜索了，一直是不动的位姿
            if(!previous_success){
                lens = 0.25 * 0.6;
                lens = lens * 1.2;
            }
            search_size<< lens, lens, lens, lens, lens, lens;

            std::cout<<"============ Pose estimation ==============\n";

            double qx;
            double qy;
            double qz;
            double qw;



            //用于R^T*(P-T)转换到相机坐标系下！
            Eigen::Matrix3d previous_global_rotation_inverse(current_global_rotation.inverse());
            Eigen::Vector3d previous_global_translation =  current_global_translation;

            //得到预测值
            current_global_rotation = init_camera_rotation_incremental * current_global_rotation;
            if(use_translation){
                current_global_translation= init_camera_translation_incremental*1000 + current_global_translation;
            }else{
                current_global_translation=  current_global_translation;
            }


            if(use_rotation){
                record_camera_rotation_incremental=init_camera_rotation_incremental;
            }
            std::cout<<"record_camera_rotation_incremental \n"<<record_camera_rotation_incremental<<"\n";
            if (use_translation){
                record_camera_trans_incremental=init_camera_translation_incremental;
            }

            //这个表示每次粒子滤波使用多少个像素点，分别是160、640、2560个点！
            int level[20] = {160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640,2560,160,640};//这个level是用来干嘛的！在ROSEFusion里是用来跳点的！

            int particle_index[20] ={0,1+20,2+40,3,4+20,5+40,6+0,7+20,8+40,
                                    9+0,10+20,11+40,12+0,13+20,14+40,
                                    15+0,16+20,17+40,18+0,19+20}; //用来对应不同的粒子数目


            int count_particle=0;//对应particle_index用于表明使用哪种粒子来搜索
            int level_index=3;
            bool success=true;
            // bool previous_success=true;

            int count=0; //记录迭代数目
            int count_success=0;
            float min_tsdf;


            //init imu data
            for (int i=0; i<imu_model.v_imu.size(); i++){
                search_data.imu_mat[2].ptr<float>(i)[0]=imu_model.v_imu[i].time;
                search_data.imu_mat[2].ptr<float>(i)[1]=imu_model.v_imu[i].dx;
                search_data.imu_mat[2].ptr<float>(i)[2]=imu_model.v_imu[i].dy;
                search_data.imu_mat[2].ptr<float>(i)[3]=imu_model.v_imu[i].dz;
                search_data.imu_mat[2].ptr<float>(i)[4]=imu_model.v_imu[i].rx;
                search_data.imu_mat[2].ptr<float>(i)[5]=imu_model.v_imu[i].ry;
                search_data.imu_mat[2].ptr<float>(i)[6]=imu_model.v_imu[i].rz;
            }

            search_data.gpu_imu_mat[2].upload(search_data.imu_mat[2]);



            while(true){

                Eigen::Matrix<double, 7, 1> mean_transform=Eigen::Matrix<double, 7, 1>::Zero();

                if(count==controller_config.max_iteration){
                    break;
                }

                if (!success){//如果此次搜索不成功就重新从最开始的粒子群开始搜索！
                    count_particle=0;
                }
                //执行一次粒子滤波，得到新的位姿和imu零偏！
                success=cuda::particle_evaluation_hash(voxelData,hashTable,volume,quaternions,search_data,current_global_rotation, current_global_translation,
                    frame_data.gpu_compact_vertex, frame_data.compact_size,
                    previous_global_rotation_inverse, previous_global_translation,
                    cam_params,particle_index[count_particle],particle_level[particle_index[count_particle]/20], //particle_level[particle_index[count_particle]/20]对应三种粒子群中的一个，表明该粒子群个数
                    search_size,level[count_particle],level_index,
                    mean_transform,&min_tsdf,imu_model,record_camera_trans_incremental,
                    record_camera_rotation_incremental,frame_global_rotation,count);


                if (count==0 && !success)
                {
                    *iter_tsdf=min_tsdf;
                }

                qx=mean_transform(4,0);
                qy=mean_transform(5,0);
                qz=mean_transform(6,0);

                if (success){
                    if (count_particle<19){
                        ++count_particle;

                    }
                    ++count_success;

                    auto camera_translation_incremental = mean_transform.head<3>();
                    qw=mean_transform(3,0);
                    Eigen::Matrix3d camera_rotation_incremental;

                    camera_rotation_incremental << 1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw),
                                                    2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw),
                                                    2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy);

                    current_global_translation =  current_global_translation + camera_translation_incremental*1000;
                    current_global_rotation = camera_rotation_incremental * current_global_rotation;

                    //用于记录最后优化成功后的增量平移和旋转
                    record_camera_rotation_incremental =camera_rotation_incremental *record_camera_rotation_incremental;
                    record_camera_trans_incremental+=camera_translation_incremental;
                }


                level_index+=311;
                int stride=static_cast<unsigned int>(std::ceil((frame_data.compact_size*1.f-level[count_particle]+1)/level[count_particle]));
                stride = stride<=0? 1: stride;

                level_index=level_index%stride;
                //更新搜索空间！
                update_seach_size(min_tsdf,controller_config.scaling_coefficient2,search_size,mean_transform);

                ++count;

            }

            if (count_success==0 ){
                previous_success = false;
                return false;

            }

            pose.block(0, 0, 3, 3) = current_global_rotation;
            pose.block(0, 3, 3, 1) = current_global_translation;
            std::cout<<"record incremental rotation: \n"<<record_camera_rotation_incremental<<std::endl;
            std::cout<<"=============record incremental translation: ==============\n"<<record_camera_trans_incremental<<std::endl;
            std::cout<<"error: \n"<<(record_camera_trans_incremental-init_camera_translation_incremental).norm()<<std::endl;


            std::cout<<"imu_model.bias_gyr:\n"<<imu_model.bias_gyr<<std::endl;
            std::cout<<"imu_model.bias_acc:\n"<<imu_model.bias_acc<<std::endl;


            previous_success = true;
            return true;
        }
        
    }
}
