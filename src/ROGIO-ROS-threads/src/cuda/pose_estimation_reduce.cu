// // Estimates the current pose using ICP
// // This is CUDA code; compile with nvcc
// // Author: Christian Diller, git@christian-diller.de

// #include "include/common.h"
// #include <iostream>
// #define BLOCK_SIZE_X 32
// #define BLOCK_SIZE_Y 32

// using Matf31da = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
// using Matf61da = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
// using Matf31fa = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
// using Matf61fa = Eigen::Matrix<float, 6, 1, Eigen::DontAlign>;

// namespace kinectfusion {
//     namespace internal {
//         namespace cuda {


//             template <unsigned int blockSize>
//             __device__ void warpReduce(volatile float *sdata, unsigned int tid) {
//                 sdata[tid] += sdata[tid + 32];
//                 sdata[tid] += sdata[tid + 16];
//                 sdata[tid] += sdata[tid + 8];
//                 sdata[tid] += sdata[tid + 4];
//                 sdata[tid] += sdata[tid + 2];
//                 sdata[tid] += sdata[tid + 1];

//             }


//             __global__
//             void particle_kernel(const PtrStepSz<short> tsdf_volume,const PtrStep<float3> vertex_map_current,
//                                     const PtrStep<float3> normal_map_current,PtrStep<float> search_value,
//                                     PtrStep<float> search_count,const Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation_current,
//                                     const Matf31fa translation_current,const Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation_previous_inv,
//                                     const Matf31fa translation_previous,const PtrStep<float> quaternion_trans,
//                                     const CameraParameters cam_params,
//                                     const int3 volume_size, const float voxel_scale, const int particle_size, const int cols,
//                                     const int rows ,const Matf61da search_size,const int level,const int level_index)
//                 {
//                 const int x = (blockIdx.x * blockDim.x + threadIdx.x)*level+level_index;
//                 const int y = (blockIdx.y * blockDim.y + threadIdx.y)*level/2+level_index/2;
//                 const int p = blockIdx.z * blockDim.z + threadIdx.z;
                
//                 // printf("%d %d =",x,y);
//                 // if (y >=rows){
//                 //     printf("%d ",y);
//                 //     // printf("aaaaa");

//                 // }
//                 // printf("%d %d\n",x,y);
//                 float point_tsdf=0;
//                 float point_count=0;
//                 if (x < cols && y <rows && p<particle_size) {
//                     Matf31fa normal_current;
//                     normal_current.x() = normal_map_current.ptr(y)[x].x;
//                     normal_current.y() = normal_map_current.ptr(y)[x].y;
//                     normal_current.z() = normal_map_current.ptr(y)[x].z;

//                     if (!isnan(normal_current.x()) && (normal_current.x()!=0 || normal_current.y()!=0 || normal_current.z()!=0)) {
                    

//                         // printf("%f %f %f\\n",normal_map_current.ptr(y)[x].x,normal_map_current.ptr(y)[x].z,normal_map_current.ptr(y)[x].y);

//                         // printf("addd");

//                         Matf31fa vertex_current;
//                         vertex_current.x() = vertex_map_current.ptr(y)[x].x;
//                         vertex_current.y() = vertex_map_current.ptr(y)[x].y;
//                         vertex_current.z() = vertex_map_current.ptr(y)[x].z;

//                         Matf31fa vertex_current_global = rotation_current * vertex_current ;

//                         const float t_x=quaternion_trans.ptr(p)[0]*search_size(0,0)*1000;
//                         const float t_y=quaternion_trans.ptr(p)[1]*search_size(1,0)*1000;
//                         const float t_z=quaternion_trans.ptr(p)[2]*search_size(2,0)*1000;
//                         //printf("t_x:%f t_y:%f t_z:%f\\n",t_x,t_y,t_z);
            
//                         const float q1=quaternion_trans.ptr(p)[3]*search_size(3,0);
//                         const float q2=quaternion_trans.ptr(p)[4]*search_size(4,0);
//                         const float q3=quaternion_trans.ptr(p)[5]*search_size(5,0);
//                         const float q0=sqrt(1-q1*q1-q2*q2-q3*q3);
//                         // if (q0==1){
//                         //     printf("%f %f %f %f\n",q0,q1,q2,q3);
//                         // }
//                         // quaternion_qw.ptr(p)[0]=q0;
//                         float q_w= -(vertex_current_global.x()*q1 + vertex_current_global.y()*q2 + vertex_current_global.z()*q3);
//                         float q_x = q0*vertex_current_global.x() - q3*vertex_current_global.y() + q2*vertex_current_global.z();
//                         float q_y = q3*vertex_current_global.x() + q0*vertex_current_global.y() - q1*vertex_current_global.z();
//                         float q_z =-q2*vertex_current_global.x() + q1*vertex_current_global.y() + q0*vertex_current_global.z();

//                         vertex_current_global.x()= q_x*q0 + q_w*(-q1) - q_z*(-q2) + q_y*(-q3) + t_x+   translation_current.x();
//                         vertex_current_global.y()= q_y*q0 + q_z*(-q1) + q_w*(-q2) - q_x*(-q3) + t_y+   translation_current.y();
//                         vertex_current_global.z()= q_z*q0 - q_y*(-q1) + q_x*(-q2) + q_w*(-q3) + t_z+   translation_current.z();
//                         // vertex_current_global.x()= q_x*q0 + q_w*(-q1) - q_z*(-q2) + q_y*(-q3) ;
//                         // vertex_current_global.y()= q_y*q0 + q_z*(-q1) + q_w*(-q2) - q_x*(-q3) ;
//                         // vertex_current_global.z()= q_z*q0 - q_y*(-q1) + q_x*(-q2) + q_w*(-q3) ;

//                         // printf("%f %f %f\n",vertex_current_global.x(),vertex_current_global.y(),vertex_current_global.z());

//                         const Matf31fa vertex_current_camera =
//                         rotation_previous_inv * (vertex_current_global - translation_previous);
//                         // printf("%f %f %f\n",vertex_current_camera.x(),vertex_current_camera.y(),vertex_current_camera.z());
//                         Eigen::Vector2i point;
//                         point.x() = __float2int_rd(
//                                 vertex_current_camera.x() * cam_params.focal_x / vertex_current_camera.z() +
//                                 cam_params.principal_x + 0.5f);
//                         point.y() = __float2int_rd(
//                                 vertex_current_camera.y() * cam_params.focal_y / vertex_current_camera.z() +
//                                 cam_params.principal_y + 0.5f);
//                         // printf("%d %d\n",point.x(),point.y());
//                         // if (vertex_current_camera.z() <0){
//                             // printf("a");
//                         // }

//                         // if (x==level_index && y==level_index){
//                         //     // printf("%f %f %f\n",vertex_current_global.x(),vertex_current_global.y(),vertex_current_global.z());
//                         //     printf("%d %d %f \n",point.x(),point.y(),search_count.ptr(p)[0]);
//                         // }




//                         if (point.x() >= 0 && point.y() >= 0 && point.x() < cols && point.y() < rows &&
//                             vertex_current_camera.z() >= 0) {
            
//                             Vec3fda grid = (vertex_current_global) / voxel_scale;

//                             if (grid.x() > 1 && grid.x() < volume_size.x - 1 && 
//                                 grid.y() > 1 && grid.y() < volume_size.y - 1 &&
//                                 grid.z() > 1 && grid.z() < volume_size.z - 1){
//                                 const float tsdf = static_cast<float>(tsdf_volume.ptr(
//                                         __float2int_rd(grid(2)) * volume_size.y + __float2int_rd(grid(1)))[__float2int_rd(grid(0))]) *
//                                                 DIVSHORTMAX;
//                                 point_tsdf=abs(tsdf);
//                                 point_count=1;
//                         }
//                     }
//                 }    
//             }



//                 __shared__ float tsdf_smem[BLOCK_SIZE_X * BLOCK_SIZE_Y];
//                 __shared__ float count_smem[BLOCK_SIZE_X * BLOCK_SIZE_Y];
//                 const int tid = threadIdx.y * blockDim.x + threadIdx.x;
//                 // printf("%d ",tid);

//                 tsdf_smem[tid]=point_tsdf;
//                 count_smem[tid]=point_count;
//                 __syncthreads();
//                 const int blockSize=1024;

//                 { if (tid < 512) { tsdf_smem[tid] += tsdf_smem[tid + 512]; count_smem[tid] += count_smem[tid + 512];} __syncthreads(); }
//                 { if (tid < 256) { tsdf_smem[tid] += tsdf_smem[tid + 256]; count_smem[tid] += count_smem[tid + 256];} __syncthreads(); }
//                 { if (tid < 128) { tsdf_smem[tid] += tsdf_smem[tid + 128]; count_smem[tid] += count_smem[tid + 128];} __syncthreads(); }
//                 { if (tid < 64) { tsdf_smem[tid] += tsdf_smem[tid + 64]; count_smem[tid] += count_smem[tid + 64];} __syncthreads(); }

//                 if (tid<32){warpReduce<64>(tsdf_smem,tid);warpReduce<64>(count_smem,tid);}
//                 if (tid==0){search_value.ptr(p)[0]=tsdf_smem[0];search_count.ptr(p)[0]=count_smem[0];}



//                 // printf("%f ",tsdf_smem[0]);
//                 // if (tid==100){
//                 //     printf("%f ",tsdf_smem[tid]);
//                 //     // printf("aaaa");
//                 // }

//                 // if (blockSize >= 1024) { if (tid < 512) { tsdf_smem[tid] += tsdf_smem[tid + 512]; } __syncthreads(); }
//                 // if (blockSize >= 512) { if (tid < 256) { tsdf_smem[tid] += tsdf_smem[tid + 256]; } __syncthreads(); }
//                 // if (blockSize >= 256) { if (tid < 128) { tsdf_smem[tid] += tsdf_smem[tid + 128];} __syncthreads(); }
//                 // if (blockSize >= 128) { if (tid < 64) { tsdf_smem[tid] += tsdf_smem[tid + 64];} __syncthreads(); }
//                 // if (tid<32){warpReduce<64>(tsdf_smem,tid);}

//                 // if (blockSize >= 1024) { if (tid < 512) { count_smem[tid] += count_smem[tid + 512]; } __syncthreads(); }
//                 // if (blockSize >= 512) { if (tid < 256) { count_smem[tid] += count_smem[tid + 256]; } __syncthreads(); }
//                 // if (blockSize >= 256) { if (tid < 128) { count_smem[tid] += count_smem[tid + 128];} __syncthreads(); }
//                 // if (blockSize >= 128) { if (tid < 64) { count_smem[tid] += count_smem[tid + 64];} __syncthreads(); }
//                 // if (tid<32){warpReduce<64>(count_smem,tid);}
//                 // if (tid==0){search_value.ptr(p)[0]=tsdf_smem[0];search_count.ptr(p)[0]=count_smem[0];}



//             }



//             bool particle_evaluation(const VolumeData& volume,const Eigen::Matrix3d& rotation_current, const Matf31da& translation_current,
//                             const cv::cuda::GpuMat& vertex_map_current, const cv::cuda::GpuMat& normal_map_current,
//                             const Eigen::Matrix3d& rotation_previous_inv, const Matf31da& translation_previous,
//                             const CameraParameters& cam_params, const int particle_size,
//                             const Matf61da& search_size,const int level,const int level_index, 
//                             Eigen::Matrix<double, 7, 1>& mean_transform, float * min_tsdf)
//             {
//                 const int cols = vertex_map_current.cols;
//                 const int rows = vertex_map_current.rows;

//                 dim3 block(BLOCK_SIZE_X, BLOCK_SIZE_Y,1);
//                 dim3 grid(1,1,1);
//                 // grid.x = static_cast<unsigned int>(std::ceil((float)cols/BLOCK_SIZE_X /level ));
//                 // grid.y = static_cast<unsigned int>(std::ceil((float)cols/BLOCK_SIZE_Y /level ));
                
                

//                 grid.z = static_cast<unsigned int>(particle_size);     


//                 // dim3 block(BLOCK_SIZE_X, BLOCK_SIZE_Y,1);
//                 // dim3 grid(1,1,1);
//                 // grid.x = static_cast<unsigned int>(std::ceil((float)particle_size / block.x));
//                 // grid.y = static_cast<unsigned int>(std::ceil((float)cols /block.y /level ));     
//                 // grid.z = static_cast<unsigned int>(std::ceil((float)rows /level));  


//                 // std::cout<<"----"<<(float)cols /level / block.y<<std::endl;
//                 // printf("%d %d\n",BLOCK_SIZE_X,BLOCK_SIZE_X);
//                 printf("%d %d %d\n",grid.x,grid.y,grid.z);
//                 cv::Mat q_trans(particle_size,6,CV_32FC1);
//                 // cv::Mat qw_trans(particle_size,1,CV_32FC1);
//                 cv::randu(q_trans,cv::Scalar(-1),cv::Scalar(1));
//                 q_trans.ptr<float>(0)[0]=0;
//                 q_trans.ptr<float>(0)[1]=0;
//                 q_trans.ptr<float>(0)[2]=0;
//                 q_trans.ptr<float>(0)[3]=0;
//                 q_trans.ptr<float>(0)[4]=0;
//                 q_trans.ptr<float>(0)[5]=0;

//                 cv::Mat search_count=cv::Mat::zeros(particle_size,1,CV_32FC1);
//                 cv::Mat search_value=cv::Mat::zeros(particle_size,1,CV_32FC1);
//                 cv::cuda::GpuMat count_buffer { cv::cuda::createContinuous(particle_size, 1, CV_32FC1) };
//                 cv::cuda::GpuMat value_buffer{ cv::cuda::createContinuous(particle_size, 1, CV_32FC1) };
//                 cv::cuda::GpuMat quaternion_trans { cv::cuda::createContinuous(particle_size, 6, CV_32FC1) };
//                 // cv::cuda::GpuMat quaternion_qw { cv::cuda::createContinuous(particle_size, 1, CV_32FC1) };
//                 clock_t time_1=clock();

//                 count_buffer.upload(search_count);
//                 value_buffer.upload(search_value);
//                 quaternion_trans.upload(q_trans);
//                 // std::cout<<rotation_previous_inv<<std::endl;
//                 // std::cout<<translation_previous<<std::endl;
//                 std::cout <<"=== CUDA transfer time"<<1000*(clock()-time_1)/(double)CLOCKS_PER_SEC<<"ms"<<std::endl;

//                 particle_kernel<<<grid,block>>>(volume.tsdf_volume,vertex_map_current,normal_map_current,
//                                             value_buffer,count_buffer,rotation_current.cast<float>(),translation_current.cast<float>(),
//                                             rotation_previous_inv.cast<float>(),translation_previous.cast<float>(),quaternion_trans,
//                                             cam_params,volume.volume_size,volume.voxel_scale,particle_size,cols,rows,search_size,
//                                             level,level_index);

//                 count_buffer.download(search_count);
//                 value_buffer.download(search_value);
//                 cudaThreadSynchronize();
//                 std::cout <<"=== CUDA particle time"<<1000*(clock()-time_1)/(double)CLOCKS_PER_SEC<<"ms"<<std::endl;

//                 // quaternion_trans.download(q_trans);
//                 // quaternion_qw.download(qw_trans);
//                 // Matf61da mean_transform=Matf61da::Zeros();
//                 double weight_sum=0;
//                 float mean_tsdf=0;
//                 double orgin_tsdf=search_value.ptr<float>(0)[0]/search_count.ptr<float>(0)[0];
//                 printf("orgin tsdf: %f\n",orgin_tsdf);
//                 printf("search count: %f ",search_count.ptr<float>(0)[0]);
//                 clock_t time_2=clock();

//                 int count_search=0;
//                 for (int i=1; i<search_count.rows ;++i){
//                     double tsdf_value=search_value.ptr<float>(i)[0]/search_count.ptr<float>(i)[0];
//                     if ( tsdf_value<orgin_tsdf){

//                             // printf("%f ",tsdf_value);

//                             const double tx=q_trans.ptr<float>(i)[0]*(double)search_size(0,0);
//                             const double ty=q_trans.ptr<float>(i)[1]*(double)search_size(1,0);
//                             const double tz=q_trans.ptr<float>(i)[2]*(double)search_size(2,0);
//                             const double qx=q_trans.ptr<float>(i)[3]*(double)search_size(3,0);
//                             const double qy=q_trans.ptr<float>(i)[4]*(double)search_size(4,0);
//                             const double qz=q_trans.ptr<float>(i)[5]*(double)search_size(5,0);
//                             // printf("tsdf weight: %f ",(orgin_tsdf-tsdf_value)/orgin_tsdf);
//                             // printf("regularzier: %f ",1/(1+abs(tx)+abs(ty)+abs(tz)+abs(qx)+abs(qy)+abs(qz))*0.005);

//                             // const double weight=(orgin_tsdf-tsdf_value)/orgin_tsdf +\
//                             //             1/(1+abs(tx)+abs(ty)+abs(tz)+abs(qx)+abs(qy)+abs(qz))*0.005;
//                             const double weight=(orgin_tsdf-tsdf_value)/orgin_tsdf;

//                             mean_transform(0,0)+=tx*weight;
//                             mean_transform(1,0)+=ty*weight;
//                             mean_transform(2,0)+=tz*weight;
                            
//                             mean_transform(4,0)+=qx*weight;
//                             mean_transform(5,0)+=qy*weight;
//                             mean_transform(6,0)+=qz*weight;
//                             const double qw=sqrt(1-qx*qx-qy*qy-qz*qz);
//                             mean_transform(3,0)+=qw*weight;
//                             // std::cout<<qw<<std::endl;
//                             weight_sum+=weight;
//                             mean_tsdf+=weight*tsdf_value;
//                             ++count_search;
//                         }
//                     // printf("%f ",search_value.ptr<float>(i)[0]);
                   
//                 }
//                 printf("mean tsdf: %f\n",mean_tsdf/weight_sum);
//                 printf("search:%d\n",count_search);
//                 std::cout <<"=== fusion trans time"<<1000*(clock()-time_2)/(double)CLOCKS_PER_SEC<<"ms"<<std::endl;


//                 if (weight_sum==0){
//                     *min_tsdf=orgin_tsdf;
//                     return false;
//                 }
//                 //normalize quaternion

//                 mean_transform=mean_transform/weight_sum;
//                 mean_tsdf=mean_tsdf/weight_sum;
//                 // std::cout<<"=====mean trans"<<mean_transform<<std::endl;

//                 double qw=mean_transform(3,0);
//                 double qx=mean_transform(4,0);
//                 double qy=mean_transform(5,0);
//                 double qz=mean_transform(6,0);
//                 double lens = 1/sqrt(qw*qw+qx*qx+qy*qy+qz*qz);
//                 // printf("norm:%lf",lens*100000000);
//                 qw=qw*lens;
//                 qx=qx*lens;
//                 qy=qy*lens;
//                 qz=qz*lens;
//                 // if (abs(qw-1)<5e-7){
//                 //     mean_transform(3,0)=1;
//                 //     mean_transform(4,0)=0;
//                 //     mean_transform(5,0)=0;
//                 //     mean_transform(6,0)=0;
//                 // }
//                 // else{

//                 mean_transform(3,0)=qw;
//                 mean_transform(4,0)=qx;
//                 mean_transform(5,0)=qy;
//                 mean_transform(6,0)=qz;
//                 // }

//                 // std::cout<<"=====mean trans2"<<mean_transform<<std::endl;
//                 *min_tsdf=mean_tsdf;
//                 // printf("%f\n",min_tsdf);
//                 return true;
//             }
//         }
//     }
// }