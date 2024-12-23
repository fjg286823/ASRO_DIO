#include <rosefusion.h>

using Matf31da = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matf61da = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Matf51da = Eigen::Matrix<double, 5, 1, Eigen::DontAlign>;
using Matf51ia = Eigen::Matrix<int, 5, 1, Eigen::DontAlign>;

using Matrix3frm = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;



namespace rosefusion {
    namespace internal {

        namespace cuda { 
            
            

            bool g_particle_evaluation(const QuaternionData& quaterinons, 
                                        SearchData& search_data ,

                                        const Matf51ia imu_data_size,

                                        const Eigen::Vector3d& init_v,
                                        const Eigen::Vector3d& init_g, 

                                        // const Eigen::Vector3d bias_acc,
                                        // const Eigen::Vector3d bias_gyr,     
                                        IMU_Model& imu_model,

                                        const int particle_index,const int particle_size,
                                        const Matf61da& search_size,
                                        Eigen::Matrix<double, 7, 1>& mean_transform, 
                                        double * min_error);


            bool bias_particle_evaluation(const QuaternionData& quaterinons, 
                                        SearchData& search_data ,

                                        const Matf51ia imu_data_size,

                                        const Eigen::Vector3d& init_v,
                                        const Eigen::Vector3d& init_g, 

                                        // const Eigen::Vector3d bias_acc,
                                        // const Eigen::Vector3d bias_gyr,     
                                        IMU_Model& imu_model,

                                        const int particle_index,const int particle_size,
                                        const Matf61da& imu_search_size,
                                        double * min_error);


        }







        namespace gravity { 

        void update_seach_size(const float error, const float scaling_coefficient,Matf61da& search_size, Eigen::Matrix<double, 7, 1>& mean_transform )
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

            search_size(3,0) = scaling_coefficient * error * normal_qx + 1e-3;
            search_size(4,0) = scaling_coefficient * error * normal_qy + 1e-3;
            search_size(5,0) = scaling_coefficient * error * normal_qz + 1e-3;  
            search_size(0,0) = scaling_coefficient * error * normal_tx + 1e-3;
            search_size(1,0) = scaling_coefficient * error * normal_ty + 1e-3;
            search_size(2,0) = scaling_coefficient * error * normal_tz + 1e-3;

            search_size(0,0) = search_size(0,0) * 5; 
            search_size(1,0) = search_size(1,0) * 5;
            search_size(2,0) = search_size(2,0) * 5;

            search_size(3,0) = search_size(3,0) ; 
            search_size(4,0) = search_size(4,0) ;
            search_size(5,0) = search_size(5,0) ;


            }

        }



        Eigen::Matrix4d inverse_trans(Eigen::Matrix4d& trans){
            Eigen::Matrix4d inv_trans;
            inv_trans.block(0,0,3,3)=trans.block(0,0,3,3).transpose();
            inv_trans.block(0,3,3,1)=-trans.block(0,0,3,3).transpose()*trans.block(0,3,3,1);
            inv_trans.row(3)<<0,0,0,1;
            return inv_trans;

        }


        //采样G对应的旋转矩阵和V来修正G,同时采样偏差进一步优化更新偏差,残差都是积分的imu结果和上一步粒子滤波优化的结果
        bool gravity_estimation(const QuaternionData& quaternions,
                              SearchData& search_data,
                              const Eigen::Matrix4d& _current_pose,
                              const Eigen::Matrix4d& _last_pose,
                              const std::vector<int> particle_level,
                              Matf61da& initialize_search_size,
                              LogConfig& log_config,
                              IMU_Model& imu_model)
        {   

            Eigen::Matrix4d current_pose=_current_pose;
            Eigen::Matrix4d last_pose=_last_pose;

            std::cout<<"============ Compute velocity and gravity ==============\n";


            //add new iframes ============================================================

            current_pose.block(0, 3, 3, 1)=current_pose.block(0, 3, 3, 1)/1000;
            last_pose.block(0, 3, 3, 1)=last_pose.block(0, 3, 3, 1)/1000;

            // std::cout<< "global_translation\n" << current_pose.block(0, 3, 3, 1)-last_pose.block(0, 3, 3, 1) <<std::endl;


            Eigen::Matrix3d relative_cam_rotation=current_pose.block(0,0,3,3) * last_pose.block(0,0,3,3).transpose(); //i ->i+1的旋转
            Eigen::Vector3d relative_cam_translation=last_pose.block(0,0,3,3).transpose() * (current_pose.block(0,3,3,1)-last_pose.block(0,3,3,1));


            // std::cout<< "relative_cam_translation\n" << relative_cam_translation <<std::endl;


            Eigen::Matrix4d relative_cam_trans = Eigen::Matrix4d::Identity();
            relative_cam_trans.block(0,0,3,3)= relative_cam_rotation;
            relative_cam_trans.block(0,3,3,1)= relative_cam_translation;

            // std::cout<<"init imu model"<<std::endl;
            Eigen::Matrix4d relative_imu_trans = inverse_trans(imu_model.imu_2_cam) * relative_cam_trans * imu_model.imu_2_cam;//imu i->i+1
            Eigen::Matrix4d w_2_imu = inverse_trans(imu_model.imu_2_cam) * inverse_trans(last_pose); //世界->imu（第i帧）

            imu_model.update_frame(w_2_imu.block(0,0,3,3), relative_imu_trans.block(0,3,3,1), last_pose); //构造interval_frame,并加入到v_iframes中！


            std::cout<<"init imu model2"<<std::endl;

            //initialization ============================================================
            Eigen::Vector3d g;
            Eigen::Vector3d v;
            if(imu_model.g_weight==0){ //这里的初始化太差了，需要后续加上一个静止初始化！
                std::cout<<imu_model.v_imu.size()<<std::endl;
                // exit(0);
                Eigen::Vector3d imu_g_direction=Eigen::Vector3d(imu_model.v_iframes[0].v_imu[0].dx, imu_model.v_iframes[0].v_imu[0].dy, imu_model.v_iframes[0].v_imu[0].dz);
                g = w_2_imu.block(0,0,3,3).transpose() * imu_g_direction.normalized()*9.8; 
                v = relative_imu_trans.block(0,3,3,1)/0.03; //这是速度？

                // std::cout<<"v:\n"<<v<<std::endl;
                // std::cout<<"relative_imu_trans:\n"<<relative_imu_trans<<std::endl;
                // std::cout<<"current_time_gap:\n"<<imu_model.current_time_gap<<std::endl;

            }else{
                g=imu_model.g;
                v=imu_model.v_iframes[0].v;
            }
            // std::cout<<"init gravity:\n"<<g<<std::endl;;
            // std::cout<<"init velocity:\n"<<v<<std::endl;

            // log_config.log_gravity<<"init gravity:\n"<<g<<"\n";
            // log_config.log_gravity<<"init velocity:\n"<<v<<"\n";

            Matf51ia imu_data_size; 
            imu_data_size<<0,0,0,0,0;
            std::cout<<"initialize imu data"<<std::endl;

            //initialize imu data (time,dx,dy,dz,rx,ry,rz)
            for (int j=0; j<imu_model.v_iframes.size();j++){
                int n_imu=imu_model.v_iframes[j].v_imu.size();
                imu_data_size(j,0)=n_imu; //表示每帧有几个imu数据
                //每4*3保存平移（i->i+1的平移）和旋转（世界到第i帧的imu旋转）
                search_data.imu_info.ptr<float>(0+j*4)[0]=imu_model.v_iframes[j].imu_translation.x(); //imu_info是[20,3]，i-i+1的平移
                search_data.imu_info.ptr<float>(0+j*4)[1]=imu_model.v_iframes[j].imu_translation.y();
                search_data.imu_info.ptr<float>(0+j*4)[2]=imu_model.v_iframes[j].imu_translation.z();

                search_data.imu_info.ptr<float>(1+j*4)[0]=imu_model.v_iframes[j].w_2_imu_rotation(0,0);
                search_data.imu_info.ptr<float>(1+j*4)[1]=imu_model.v_iframes[j].w_2_imu_rotation(0,1);
                search_data.imu_info.ptr<float>(1+j*4)[2]=imu_model.v_iframes[j].w_2_imu_rotation(0,2);

                search_data.imu_info.ptr<float>(2+j*4)[0]=imu_model.v_iframes[j].w_2_imu_rotation(1,0);
                search_data.imu_info.ptr<float>(2+j*4)[1]=imu_model.v_iframes[j].w_2_imu_rotation(1,1);
                search_data.imu_info.ptr<float>(2+j*4)[2]=imu_model.v_iframes[j].w_2_imu_rotation(1,2);

                search_data.imu_info.ptr<float>(3+j*4)[0]=imu_model.v_iframes[j].w_2_imu_rotation(2,0);
                search_data.imu_info.ptr<float>(3+j*4)[1]=imu_model.v_iframes[j].w_2_imu_rotation(2,1);
                search_data.imu_info.ptr<float>(3+j*4)[2]=imu_model.v_iframes[j].w_2_imu_rotation(2,2);


                for (int i=0; i<n_imu; i++){ //每个帧间的imu数据
                    search_data.imu_mat[j].ptr<float>(i)[0]=imu_model.v_iframes[j].v_imu[i].time;
                    search_data.imu_mat[j].ptr<float>(i)[1]=imu_model.v_iframes[j].v_imu[i].dx;
                    search_data.imu_mat[j].ptr<float>(i)[2]=imu_model.v_iframes[j].v_imu[i].dy;
                    search_data.imu_mat[j].ptr<float>(i)[3]=imu_model.v_iframes[j].v_imu[i].dz;
                    search_data.imu_mat[j].ptr<float>(i)[4]=imu_model.v_iframes[j].v_imu[i].rx;
                    search_data.imu_mat[j].ptr<float>(i)[5]=imu_model.v_iframes[j].v_imu[i].ry;
                    search_data.imu_mat[j].ptr<float>(i)[6]=imu_model.v_iframes[j].v_imu[i].rz;
                }

                search_data.gpu_imu_mat[j].upload(search_data.imu_mat[j]);
            }
            search_data.gpu_imu_info.upload(search_data.imu_info);

            // std::cout<<"imu_data_size:\n"<<imu_data_size;


            Matf61da search_size;

            // float lens= controller_config.scaling_coefficient1*(*iter_tsdf);
            float lens=0.35;
            search_size<< lens*5, lens*5, lens*5, lens, lens, lens;

            
            //update initialize pose 
            double qx;
            double qy;
            double qz;
            double qw;


            // int particle_index[20] ={0+40,1+20,2+40,3,4+20,5+40,6+0,7+20,8+40,
            //                         9+0,10+20,11+40,12+0,13+20,14+40,
            //                         15+0,16+20,17+40,18+0,19+20};

            int particle_index[20] ={0,1+40,2+40,3+40,4+20,5+40,6+40,7+40,8+40,
                                    9+40,10+40,11+40,12+40,13+40,14+40,
                                    15+40,16+40,17+40,18+40,19+40};




            int count_particle=0;
            bool success_gravity=true;
            bool success_bias=true;

            int count=0;
            int count_success=0;
            double min_error;


            while(true){
                Eigen::Matrix<double, 7, 1> mean_transform=Eigen::Matrix<double, 7, 1>::Zero();
  
                if(count==3){  //只迭代3次
                    break;
                }

                if (!success_gravity||!success_bias){ //如果搜索不成功就重新开始
                    count_particle=0;
                }

                // update gravity 更新v 和g对应的旋转矩阵
                success_gravity=cuda::g_particle_evaluation(quaternions,search_data,
                                    imu_data_size, //表示每帧几个imu数据
                                    v,g,
                                    imu_model,
                                    particle_index[count_particle],
                                    particle_level[particle_index[count_particle]/20], //表示粒子群个数
                                    search_size, mean_transform, &min_error); 

                qx=mean_transform(4,0);
                qy=mean_transform(5,0);
                qz=mean_transform(6,0);
                // std::cout<<"imu_model v:\n"<<v<<std::endl;
                // std::cout<<"imu_model g:\n"<<g<<std::endl;

                if (success_gravity){


                    auto v_incremental = mean_transform.head<3>();
                    qw=mean_transform(3,0);
                    Eigen::Matrix3d g_rotation_incremental;

                    g_rotation_incremental << 1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw),
                                            2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw),
                                            2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy);

                    v = v_incremental + v;
                    g = g_rotation_incremental * g;
                }




                //这里面会放大对速度的搜索
                gravity::update_seach_size(min_error, 0.2, search_size, mean_transform);
                
                //====================================================================
                //update bias 

                // if (count>15){
                    double min_imu_error; 
                    Matf61da imu_search_size;
                    // imu_search_size<<0.0001,0.0001,0.0001,0.0001,0.0001,0.0001;
                    imu_search_size<<0.0005,0.0005,0.0005,0.0001,0.0001,0.0001;
                    // 采样bias执行imu积分基于之前估计好的相对平移作差得到残差，残差是连续3帧的和，进行粒子滤波一次得到更好的偏差
                    success_gravity=cuda::bias_particle_evaluation(quaternions,search_data,
                                        imu_data_size,
                                        v,g,
                                        imu_model,
                                        particle_index[count_particle],
                                        particle_level[particle_index[count_particle]/20],
                                        imu_search_size, &min_imu_error); 
                // }

                // std::cout<<"imu_model v:\n"<<v<<std::endl;
                // std::cout<<"imu_model g:\n"<<g<<std::endl;
                ++count_particle;
                ++count;
                // exit(0);
                if (success_bias||success_gravity){
                    ++count_success;
                }

            }
            // exit(0);
            imu_model.update_gravity(g,min_error); //就是加权求和更新！

            imu_model.v_iframes[0].v=v;

            imu_model.update_velocity();//由于窗口中的第一帧v更新了，偏差也更新了，重新计算得到每一帧的v

            // std::cout<<"predict v:\n"<<imu_model.latest_v<<std::endl;
            // std::cout<<"predict g:\n"<<imu_model.g<<std::endl;

            // std::cout<<"imu_model.bias_gyra:\n"<<imu_model.bias_gyr<<std::endl;
            // std::cout<<"imu_model.bias_acca:\n"<<imu_model.bias_acc<<std::endl;
            // exit(0);
            if (count_success==0 ){
                return false;
            }


            return true;
            
        }
    }
}
