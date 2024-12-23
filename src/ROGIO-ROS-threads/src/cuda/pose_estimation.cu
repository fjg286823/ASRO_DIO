#include "include/common.h"
#include "ITMVoxelBlockHash.h"
#include "ITMVoxelTypes.h"
#include "ITMRepresentationAccess.h"

#define BLOCK_SIZE_X 32
#define BLOCK_SIZE_Y 32

using Matf31da = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matf61da = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Matf31fa = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
using Matf61fa = Eigen::Matrix<float, 6, 1, Eigen::DontAlign>;

namespace rosefusion
{
    namespace internal
    {
        namespace cuda
        {
            __global__ void particle_kernel_tsdf_hash(const ITMVoxel_d *voxelData, const ITMHashEntry *hashTable,
                                                      const PtrStep<float3> compact_vertex_current, const int compact_size, PtrStep<int> search_value,
                                                      PtrStep<int> search_count, const Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation_current,
                                                      const Matf31fa translation_current, const Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation_previous_inv,
                                                      const Matf31fa translation_previous, const PtrStep<float> quaternion_trans,
                                                      const CameraParameters cam_params,
                                                      const float voxel_scale, const int particle_size, const int cols,
                                                      const int rows, const Matf61da search_size, const int level, const int level_index)
            {

                const int p = blockIdx.x * blockDim.x + threadIdx.x;                         // 粒子索引
                const int x = (blockIdx.y * blockDim.y + threadIdx.y) * level + level_index; // 得到稠密点云的索引，

                if (x >= compact_size || p >= particle_size)
                {
                    return;
                }

                Matf31fa vertex_current;
                vertex_current.x() = compact_vertex_current.ptr(x)[0].x;
                vertex_current.y() = compact_vertex_current.ptr(x)[0].y;
                vertex_current.z() = compact_vertex_current.ptr(x)[0].z;
                // printf("%d %f %f %f\n",x,vertex_current.x(),vertex_current.y(),vertex_current.z());
                // return;
                Matf31fa vertex_current_global = rotation_current * vertex_current;
                // printf("%f %f %f\n",vertex_current.x(),vertex_current.y(),vertex_current.z());

                const float t_x = quaternion_trans.ptr(p)[0] * search_size(0, 0) * 1000;
                const float t_y = quaternion_trans.ptr(p)[1] * search_size(1, 0) * 1000;
                const float t_z = quaternion_trans.ptr(p)[2] * search_size(2, 0) * 1000;

                const float q1 = quaternion_trans.ptr(p)[3] * search_size(3, 0);
                const float q2 = quaternion_trans.ptr(p)[4] * search_size(4, 0);
                const float q3 = quaternion_trans.ptr(p)[5] * search_size(5, 0);
                const float q0 = sqrt(1 - q1 * q1 - q2 * q2 - q3 * q3);

                float q_w = -(vertex_current_global.x() * q1 + vertex_current_global.y() * q2 + vertex_current_global.z() * q3);
                float q_x = q0 * vertex_current_global.x() - q3 * vertex_current_global.y() + q2 * vertex_current_global.z();
                float q_y = q3 * vertex_current_global.x() + q0 * vertex_current_global.y() - q1 * vertex_current_global.z();
                float q_z = -q2 * vertex_current_global.x() + q1 * vertex_current_global.y() + q0 * vertex_current_global.z();

                vertex_current_global.x() = q_x * q0 + q_w * (-q1) - q_z * (-q2) + q_y * (-q3) + t_x + translation_current.x();
                vertex_current_global.y() = q_y * q0 + q_z * (-q1) + q_w * (-q2) - q_x * (-q3) + t_y + translation_current.y();
                vertex_current_global.z() = q_z * q0 - q_y * (-q1) + q_x * (-q2) + q_w * (-q3) + t_z + translation_current.z();

                const Matf31fa vertex_current_camera =
                    rotation_previous_inv * (vertex_current_global - translation_previous); // 得到图像坐标系下的坐标

                Eigen::Vector2i point;
                point.x() = __float2int_rd(
                    vertex_current_camera.x() * cam_params.focal_x / vertex_current_camera.z() +
                    cam_params.principal_x + 0.5f);
                point.y() = __float2int_rd(
                    vertex_current_camera.y() * cam_params.focal_y / vertex_current_camera.z() +
                    cam_params.principal_y + 0.5f);

                // printf(" %d %d--",point.x(), point.y());
                if (point.x() >= 0 && point.y() >= 0 && point.x() < cols && point.y() < rows &&
                    vertex_current_camera.z() >= 0)
                {

                    Vec3fda grid = (vertex_current_global) / voxel_scale;

                    int vmIndex = 0;
                    int tsdf = static_cast<int>(readFromSDF_float_uninterpolated(voxelData, hashTable, grid, vmIndex));

                    atomicAdd_system(search_value + p, abs(tsdf));

                    // if(vmIndex)
                    //     printf("grid %f %f %f,tsdf:  %d\n",grid.x(),grid.y(),grid.z(),abs(tsdf));
                    atomicAdd_system(search_count + p, 1);
                }
            }
            // 根据候选位姿搜索得到对应的tsdf残差
            __global__ void particle_kernel_tsdf(const PtrStepSz<short> tsdf_volume,
                                                 const PtrStep<float3> compact_vertex_current, const int compact_size, PtrStep<int> search_value,
                                                 PtrStep<int> search_count, const Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation_current,
                                                 const Matf31fa translation_current, const Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation_previous_inv,
                                                 const Matf31fa translation_previous, const PtrStep<float> quaternion_trans,
                                                 const CameraParameters cam_params,
                                                 const int3 volume_size, const float voxel_scale, const int particle_size, const int cols,
                                                 const int rows, const Matf61da search_size, const int level, const int level_index)
            {
                const int p = blockIdx.x * blockDim.x + threadIdx.x;                         // 粒子索引
                const int x = (blockIdx.y * blockDim.y + threadIdx.y) * level + level_index; // 得到稠密点云的索引

                if (x >= compact_size || p >= particle_size)
                {
                    return;
                }

                Matf31fa vertex_current;
                vertex_current.x() = compact_vertex_current.ptr(x)[0].x;
                vertex_current.y() = compact_vertex_current.ptr(x)[0].y;
                vertex_current.z() = compact_vertex_current.ptr(x)[0].z;
                // printf("%d %f %f %f\n",x,vertex_current.x(),vertex_current.y(),vertex_current.z());
                // return;
                Matf31fa vertex_current_global = rotation_current * vertex_current;
                // printf("%f %f %f\n",vertex_current.x(),vertex_current.y(),vertex_current.z());

                const float t_x = quaternion_trans.ptr(p)[0] * search_size(0, 0) * 1000;
                const float t_y = quaternion_trans.ptr(p)[1] * search_size(1, 0) * 1000;
                const float t_z = quaternion_trans.ptr(p)[2] * search_size(2, 0) * 1000;

                const float q1 = quaternion_trans.ptr(p)[3] * search_size(3, 0);
                const float q2 = quaternion_trans.ptr(p)[4] * search_size(4, 0);
                const float q3 = quaternion_trans.ptr(p)[5] * search_size(5, 0);
                const float q0 = sqrt(1 - q1 * q1 - q2 * q2 - q3 * q3);

                float q_w = -(vertex_current_global.x() * q1 + vertex_current_global.y() * q2 + vertex_current_global.z() * q3);
                float q_x = q0 * vertex_current_global.x() - q3 * vertex_current_global.y() + q2 * vertex_current_global.z();
                float q_y = q3 * vertex_current_global.x() + q0 * vertex_current_global.y() - q1 * vertex_current_global.z();
                float q_z = -q2 * vertex_current_global.x() + q1 * vertex_current_global.y() + q0 * vertex_current_global.z();

                vertex_current_global.x() = q_x * q0 + q_w * (-q1) - q_z * (-q2) + q_y * (-q3) + t_x + translation_current.x();
                vertex_current_global.y() = q_y * q0 + q_z * (-q1) + q_w * (-q2) - q_x * (-q3) + t_y + translation_current.y();
                vertex_current_global.z() = q_z * q0 - q_y * (-q1) + q_x * (-q2) + q_w * (-q3) + t_z + translation_current.z();

                const Matf31fa vertex_current_camera =
                    rotation_previous_inv * (vertex_current_global - translation_previous); // 得到图像坐标系下的坐标

                Eigen::Vector2i point;
                point.x() = __float2int_rd(
                    vertex_current_camera.x() * cam_params.focal_x / vertex_current_camera.z() +
                    cam_params.principal_x + 0.5f);
                point.y() = __float2int_rd(
                    vertex_current_camera.y() * cam_params.focal_y / vertex_current_camera.z() +
                    cam_params.principal_y + 0.5f);

                // printf(" %d %d--",point.x(), point.y());
                if (point.x() >= 0 && point.y() >= 0 && point.x() < cols && point.y() < rows &&
                    vertex_current_camera.z() >= 0)
                {

                    Vec3fda grid = (vertex_current_global) / voxel_scale;

                    if (grid.x() < 1 || grid.x() >= volume_size.x - 1 || grid.y() < 1 ||
                        grid.y() >= volume_size.y - 1 ||
                        grid.z() < 1 || grid.z() >= volume_size.z - 1)
                    {
                        return;
                    }

                    int tsdf = static_cast<int>(tsdf_volume.ptr(
                        __float2int_rd(grid(2)) * volume_size.y + __float2int_rd(grid(1)))[__float2int_rd(grid(0))]);

                    atomicAdd_system(search_value + p, abs(tsdf));
                    // printf("%d", abs(tsdf));
                    atomicAdd_system(search_count + p, 1);
                }
            }

            namespace fitness
            {
                __device__
                    double4
                    deltaQ(const Eigen::Vector3d &theta)
                {
                    // typedef typename Derived::Scalar Scalar_t;

                    // Eigen::Quaternion<float> dq;
                    Eigen::Matrix<double, 3, 1> half_theta = theta;
                    half_theta /= static_cast<float>(2.0);
                    // dq.w() = static_cast<float>(1.0);
                    // dq.x() = half_theta.x();
                    // dq.y() = half_theta.y();
                    // dq.z() = half_theta.z();
                    return make_double4(half_theta.x(), half_theta.y(), half_theta.z(), static_cast<double>(1.0));
                }

                __device__
                    double4
                    quaternion_multiply(double4 a, double4 b)
                {

                    return make_double4(
                        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
                }

                __device__
                    Eigen::Vector3d
                    quaternion_rotation(double4 q, Eigen::Vector3d b)
                {
                    double4 q_b = make_double4(b.x(), b.y(), b.z(), 0);
                    double4 q_conj = make_double4(-q.x, -q.y, -q.z, q.w);
                    double4 q_temp = quaternion_multiply(q, q_b);
                    double4 q_result = quaternion_multiply(q_temp, q_conj);

                    return Eigen::Vector3d(q_result.x, q_result.y, q_result.z);
                }

                __device__
                    Eigen::Matrix3d
                    quaternion_2_rm(double4 q)
                {
                    Eigen::Matrix3d rm;
                    double qw = q.w;
                    double qx = q.x;
                    double qy = q.y;
                    double qz = q.z;

                    rm << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
                        2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
                        2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy);
                    return rm;
                }

                __device__
                    Eigen::Matrix4d
                    inverse_trans(const Eigen::Matrix4d trans)
                {
                    Eigen::Matrix4d inv_trans;
                    inv_trans.block(0, 0, 3, 3) = trans.block(0, 0, 3, 3).transpose();
                    inv_trans.block(0, 3, 3, 1) = -trans.block(0, 0, 3, 3).transpose() * trans.block(0, 3, 3, 1);
                    inv_trans.row(3) << 0, 0, 0, 1;
                    return inv_trans;
                }
                // 将两帧之间的数据进行积分得到 P,Q,V
                __device__ void intergration_time_frame(const Eigen::Vector3d g,
                                                        const Eigen::Matrix3d w_2_imu_rotation, Eigen::Vector3d imu_sample_v, // imu_sample_v初始为0速度
                                                        const PtrStep<float> imu_data, const int n_imu,
                                                        Eigen::Vector3d &latest_P,
                                                        double4 &latest_Q,
                                                        Eigen::Vector3d sample_bias_acc,
                                                        Eigen::Vector3d sample_bias_gyr

                )
                {

                    // Eigen::Vector3d latest_P(0,0,0);
                    Eigen::Vector3d imu_g = w_2_imu_rotation * g;
                    // Eigen::Vector3d imu_v=imu_sample_v;

                    double latest_time = imu_data.ptr(0)[0];
                    Eigen::Vector3d latest_acc(imu_data.ptr(0)[1], imu_data.ptr(0)[2], imu_data.ptr(0)[3]);
                    Eigen::Vector3d latest_gyr(imu_data.ptr(0)[4], imu_data.ptr(0)[5], imu_data.ptr(0)[6]);
                    latest_acc -= sample_bias_acc;
                    latest_gyr -= sample_bias_gyr;

                    // double4 latest_Q=make_double4(0,0,0,1);

                    // printf("imu v2: %f %f %f\n",imu_sample_v.x(),imu_sample_v.y(),imu_sample_v.z());
                    Eigen::Vector3d un_acc_0;
                    Eigen::Vector3d un_gyr;
                    Eigen::Vector3d un_acc_1;
                    Eigen::Vector3d un_acc;
                    Eigen::Vector3d acc;
                    Eigen::Vector3d gyr;
                    for (int i = 1; i < n_imu; i++)
                    {
                        // printf("iter: %d\n",i);
                        acc << imu_data.ptr(i)[1], imu_data.ptr(i)[2], imu_data.ptr(i)[3];
                        gyr << imu_data.ptr(i)[4], imu_data.ptr(i)[5], imu_data.ptr(i)[6];
                        acc = acc - sample_bias_acc;
                        gyr = gyr - sample_bias_gyr;
                        double t = imu_data.ptr(i)[0];
                        double dt = t - latest_time;
                        // printf("time:%f %f %f\n",t,latest_time,dt);

                        un_acc_0 = quaternion_rotation(latest_Q, latest_acc) - imu_g;
                        un_gyr = 0.5 * (latest_gyr + gyr);

                        latest_Q = quaternion_multiply(latest_Q, deltaQ(un_gyr * dt));
                        un_acc_1 = quaternion_rotation(latest_Q, acc) - imu_g;

                        un_acc = 0.5 * (un_acc_0 + un_acc_1);

                        latest_P = latest_P + dt * imu_sample_v + 0.5 * dt * dt * un_acc;
                        imu_sample_v = imu_sample_v + dt * un_acc;

                        latest_acc = acc;
                        latest_gyr = gyr;

                        latest_time = t;
                    }

                    // return latest_P;
                }

            }
            // 这个平移残差和旋转残差是通过随机采样零偏，再积分imu得到 P,Q,再与选定的候选粒子的偏差，即增量位姿做差得到的！
            __global__ void particle_evaluation_imu(

                const PtrStep<float> imu_data, // 帧间的imu数据
                const Eigen::Matrix3d frame_rotation,
                const Eigen::Matrix3d incre_rotation,
                const Eigen::Vector3d incre_translation,
                const Eigen::Vector3d init_v, const Eigen::Vector3d init_g,
                const Eigen::Vector3d bias_acc, const Eigen::Vector3d bias_gyr,
                const Eigen::Matrix4d imu_2_cam,
                const Eigen::Matrix3d w_2_imu_rotation,
                PtrStep<float> search_value,
                const PtrStep<float> quaternion_trans, // 搜索位姿的粒子群
                const PtrStep<float> disturb_imu_bias, // 搜索imu的粒子群
                const int particle_size,               // particle_size对应三种粒子群个数中的一个，表明该粒子群个数
                const int imu_data_size,
                const Matf61da search_size_pose, const Matf61da search_size_imu) // 分别是位姿的缩放尺度和imu状态的缩放尺度
            {

                const int p = blockIdx.x * blockDim.x + threadIdx.x; // 粒子索引
                if (p >= particle_size)
                {
                    return;
                }

                // if(p!=100){
                //     return;
                // }

                const double bias_acc_x = bias_acc.x() + (double)disturb_imu_bias.ptr(p)[0] * search_size_imu(0, 0);
                const double bias_acc_y = bias_acc.y() + (double)disturb_imu_bias.ptr(p)[1] * search_size_imu(1, 0);
                const double bias_acc_z = bias_acc.z() + (double)disturb_imu_bias.ptr(p)[2] * search_size_imu(2, 0);
                Eigen::Vector3d sample_bias_acc(bias_acc_x, bias_acc_y, bias_acc_z);
                const double bias_gyr_x = bias_gyr.x() + (double)disturb_imu_bias.ptr(p)[3] * search_size_imu(3, 0);
                const double bias_gyr_y = bias_gyr.y() + (double)disturb_imu_bias.ptr(p)[4] * search_size_imu(4, 0);
                const double bias_gyr_z = bias_gyr.z() + (double)disturb_imu_bias.ptr(p)[5] * search_size_imu(5, 0);
                Eigen::Vector3d sample_bias_gyr(bias_gyr_x, bias_gyr_y, bias_gyr_z);

                // printf("acc_bias: %f %f %f\n",sample_bias_acc.x(),sample_bias_acc.y(),sample_bias_acc.z());
                // printf("gyr_bias: %f %f %f\n",sample_bias_gyr.x(),sample_bias_gyr.y(),sample_bias_gyr.z());

                const double t_x = quaternion_trans.ptr(p)[0] * search_size_pose(0, 0);
                const double t_y = quaternion_trans.ptr(p)[1] * search_size_pose(1, 0);
                const double t_z = quaternion_trans.ptr(p)[2] * search_size_pose(2, 0);

                const double q1 = quaternion_trans.ptr(p)[3] * search_size_pose(3, 0);
                const double q2 = quaternion_trans.ptr(p)[4] * search_size_pose(4, 0);
                const double q3 = quaternion_trans.ptr(p)[5] * search_size_pose(5, 0);
                const double q0 = sqrt(1 - q1 * q1 - q2 * q2 - q3 * q3);
                // printf("t: %f %f %f\n",t_x,t_y,t_z);
                // printf("q: %f %f %f\n",sample_bias_gyr.x(),sample_bias_gyr.y(),sample_bias_gyr.z());

                // 得到增量的旋转和平移
                double4 disturb_quaternion = make_double4(q1, q2, q3, q0);
                Eigen::Matrix3d relative_rotation = fitness::quaternion_2_rm(disturb_quaternion) * incre_rotation;
                Eigen::Vector3d relative_translation = Eigen::Vector3d(t_x, t_y, t_z) + incre_translation;

                // imu
                Eigen::Vector3d imu_transaltion(0, 0, 0);
                double4 imu_quaternion = make_double4(0, 0, 0, 1);
                // 将两帧之间的数据进行积分得到 P,Q,V，均是imu坐标系下的，其实是上一帧imu坐标系下！
                fitness::intergration_time_frame(init_g, w_2_imu_rotation, init_v, imu_data, imu_data_size, imu_transaltion, imu_quaternion,
                                                 sample_bias_acc, sample_bias_gyr);

                // imu to world
                Eigen::Matrix4d imu_realtive_motion = Eigen::Matrix4d::Identity();
                imu_realtive_motion.block(0, 0, 3, 3) = fitness::quaternion_2_rm(imu_quaternion);
                imu_realtive_motion.block(0, 3, 3, 1) = imu_transaltion;
                Eigen::Matrix4d cam_relative_motion = imu_2_cam * imu_realtive_motion * fitness::inverse_trans(imu_2_cam);
                Eigen::Vector3d global_trans = frame_rotation * cam_relative_motion.block(0, 3, 3, 1);

                // error
                double error_translation = (global_trans - relative_translation).norm();

                Eigen::Matrix3d differ_rotation = cam_relative_motion.block(0, 0, 3, 3) * relative_rotation.transpose();

                // printf("predict differ_rotation: %f %f %f\n",differ_rotation(0,0),differ_rotation(0,1),differ_rotation(0,2));
                // printf("predict differ_rotation: %f %f %f\n",differ_rotation(1,0),differ_rotation(1,1),differ_rotation(2,2));
                // printf("predict differ_rotation: %f %f %f\n",differ_rotation(2,0),differ_rotation(2,1),differ_rotation(2,2));

                // printf("error_angle: %f\n",(differ_rotation(0,0)+differ_rotation(1,1)+differ_rotation(2,2)-1)/2);
                double error_rotation = 1 - ((differ_rotation(0, 0) + differ_rotation(1, 1) + differ_rotation(2, 2) - 1) / 2);

                // printf("error1: %f     ",error_rotation);
                // printf("error2: %f\n",error_translation);

                search_value.ptr(p)[0] = 0.5 * error_translation + error_rotation;
                // search_value.ptr(p)[0]=error_rotation;
            }

            // 用tsdf残差和平移、旋转残差作为总残差进行优化
            bool particle_evaluation(const VolumeData &volume,
                                     const QuaternionData &quaterinons, SearchData &search_data, const Eigen::Matrix3d &rotation_current, const Matf31da &translation_current,
                                     const cv::cuda::GpuMat &gpu_compact_vertex, int compact_size,
                                     const Eigen::Matrix3d &rotation_previous_inv, const Matf31da &translation_previous,
                                     const CameraParameters &cam_params, const int particle_index, const int particle_size, // particle_index对应使用哪种粒子群（不同群的候选位姿和粒子个数不同）particle_size对应三种粒子群个数中的一个，表明该粒子群个数
                                     const Matf61da &search_size, int level, const int level_index,
                                     Eigen::Matrix<double, 7, 1> &mean_transform, float *min_tsdf, IMU_Model &imu_model,
                                     const Eigen::Vector3d incre_translation, const Eigen::Matrix3d incre_rotation,
                                     const Eigen::Matrix3d frame_rotation, int iter_times)

            {
                std::cout.precision(17);
                // std::cout<<"begin iteration"<<std::endl;
                const int cols = cam_params.image_width;
                const int rows = cam_params.image_height;

                dim3 block(BLOCK_SIZE_X * BLOCK_SIZE_Y, 1, 1);
                dim3 grid(1, 1, 1);
                grid.x = static_cast<unsigned int>(std::ceil((float)particle_size / block.y / block.x));
                // level = level * level ;
                // (frame_data.compact_size-159)/160
                grid.y = level;                                  // 这个level干嘛的
                int stride = (compact_size - level + 1) / level; // 步长
                // std::cout<<"level:"<<level<<"  stride:"<<stride<<"  level_index"<<level_index<<std::endl;

                // grid.y = static_cast<unsigned int>(std::ceil((float)compact_size /level));
                // grid.z = static_cast<unsigned int>(std::ceil((float)rows /level));

                search_data.gpu_search_count[particle_index / 20].setTo(0);
                search_data.gpu_search_value[particle_index / 20].setTo(0);
                // 采样候选位姿，得到一个残差
                particle_kernel_tsdf<<<grid, block>>>(volume.tsdf_volume, gpu_compact_vertex, compact_size,
                                                      search_data.gpu_search_value[particle_index / 20],
                                                      search_data.gpu_search_count[particle_index / 20],
                                                      rotation_current.cast<float>(), translation_current.cast<float>(),
                                                      rotation_previous_inv.cast<float>(), translation_previous.cast<float>(), quaterinons.q[particle_index], // 粒子群，即候选位姿
                                                      cam_params, volume.volume_size, volume.voxel_scale, particle_size, cols, rows, search_size,
                                                      stride, level_index);
                cv::Mat search_data_count = search_data.search_count[particle_index / 20];
                cv::Mat search_data_value = search_data.search_value[particle_index / 20];
                search_data.gpu_search_count[particle_index / 20].download(search_data_count);
                search_data.gpu_search_value[particle_index / 20].download(search_data_value);

                if ((double)search_data_count.ptr<int>(0)[0] < 10) // 说明原始图像可用点！
                {
                    std::cout << "no depth tsdf res: " << (double)search_data_value.ptr<int>(0)[0] << std::endl;
                }

                dim3 block2(BLOCK_SIZE_X, 1, 1);
                dim3 grid2(1, 1, 1);
                grid2.x = static_cast<unsigned int>(std::ceil((double)particle_size / block.y));

                // const PtrStep<float> imu_data,
                // const Matrix3d current_rotation,
                // const Eigen::Matrix<float, 3, 3, Eigen::DontAlign> incre_rotation,
                // const Matf31fa incre_translation,
                // const Matf31da init_v, const Matf31da init_g,

                // const Matrix4d imu_2_cam,
                // const Matrix3d w_imu_rotation,
                // PtrStep<float> search_value,
                // const PtrStep<float> quaternion_trans,
                // const PtrStep<float> disturb_imu_bias,
                // const int particle_size,
                // const int imu_data_size,
                // const Matf61da search_size_pose, const Matf61da search_size_imu){

                // int v_iframes_size=imu_model.v_iframes.size();
                // interval_frame iframe=imu_model.v_iframes[imu_model.v_iframes.size()-1];
                Eigen::Matrix3d w_2_imu_rotation = imu_model.imu_2_cam_rotation.transpose() * frame_rotation.transpose(); // 世界->imu的旋转！，是在上一帧的基础下的
                int pst_type = particle_index / 20;                                                                       // 表示是10240 3072 1024中的那个粒子个数
                int new_particle_index = ((particle_index + 1) % 20) + pst_type * 20;                                     // 这是当前粒子数的下一个粒子数索引！

                Matf61da imu_search_size;
                imu_search_size << 0.0005, 0.0005, 0.0005, 0.0001, 0.0001, 0.0001;

                int threshold_usage_imu = 15;
                double weight_imu;
                cv::Mat imu_search_value = search_data.imu_search_value[particle_index / 20];

                if (iter_times < threshold_usage_imu) // iter_times迭代次数
                {
                    weight_imu = 0;
                }
                else
                {
                    // weight_imu=0;
                    weight_imu = 0.1;

                    search_data.gpu_imu_search_value[particle_index / 20].setTo(0);
                    // 这个平移残差和旋转残差是通过随机采样零偏，再积分imu得到 P,Q,再与选定的候选粒子的偏差，即增量位姿做差得到的！
                    particle_evaluation_imu<<<grid2, block2>>>(search_data.gpu_imu_mat[2], //(gpu_imu_mat[2] = 100*7)
                                                               frame_rotation,
                                                               incre_rotation, incre_translation, // 当前相机坐标系下的增量
                                                               imu_model.latest_v, imu_model.g,
                                                               imu_model.bias_acc, imu_model.bias_gyr,
                                                               imu_model.imu_2_cam, w_2_imu_rotation,
                                                               search_data.gpu_imu_search_value[particle_index / 20],
                                                               quaterinons.q[particle_index],
                                                               quaterinons.q[new_particle_index],
                                                               particle_size, // particle_size对应三种粒子群个数中的一个，表明该粒子群个数
                                                               imu_model.v_imu.size(),
                                                               search_size,
                                                               imu_search_size);
                    search_data.gpu_imu_search_value[particle_index / 20].download(imu_search_value);
                }

                cudaDeviceSynchronize();

                clock_t time_2 = clock();

                int count_search = 0;
                const int iter_rows = particle_size;

                double sum_t_x = 0.0;
                double sum_t_y = 0.0;
                double sum_t_z = 0.0;
                double sum_q_x = 0.0;
                double sum_q_y = 0.0;
                double sum_q_z = 0.0;
                double sum_q_w = 0.0;

                double sum_bias_acc_x = 0.0;
                double sum_bias_acc_y = 0.0;
                double sum_bias_acc_z = 0.0;
                double sum_bias_gyr_x = 0.0;
                double sum_bias_gyr_y = 0.0;
                double sum_bias_gyr_z = 0.0;

                double sum_weight_sum = 0.0;
                double sum_mean_fitness = 0.0;

                double weight_tsdf = 1;

                double orgin_tsdf = (double)search_data_value.ptr<int>(0)[0] / (double)search_data_count.ptr<int>(0)[0] * DIVSHORTMAX;
                int orgin_count = search_data_count.ptr<int>(0)[0];

                double orgin_imu_error = (double)imu_search_value.ptr<float>(0)[0];

                double orgin_fitness = orgin_tsdf * weight_tsdf + orgin_imu_error * weight_imu;
                // printf("orgin:%f %f %f %d\n",orgin_fitness,orgin_tsdf,orgin_imu_error,orgin_count);

                for (int i = 1; i < iter_rows; ++i) // 遍历所有粒子！
                {

                    double tsdf_value = (double)search_data_value.ptr<int>(i)[0] / (double)search_data_count.ptr<int>(i)[0] * DIVSHORTMAX;
                    double imu_error_value = (double)imu_search_value.ptr<float>(i)[0];
                    double fitness = tsdf_value * weight_tsdf + imu_error_value * weight_imu;
                    // printf("%f %f %f\n",fitness,tsdf_value,imu_error_value);

                    if (fitness < orgin_fitness && ((search_data_count.ptr<int>(i)[0]) > (orgin_count / 2.0)))
                    {

                        const double tx = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[0];
                        const double ty = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[1];
                        const double tz = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[2];
                        double qx = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[3];
                        double qy = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[4];
                        double qz = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[5];

                        const double disturb_imu_bias_acc_x = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[0];
                        const double disturb_imu_bias_acc_y = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[1];
                        const double disturb_imu_bias_acc_z = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[2];
                        const double disturb_imu_bias_gyr_x = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[3];
                        const double disturb_imu_bias_gyr_y = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[4];
                        const double disturb_imu_bias_gyr_z = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[5];

                        const double weight = (orgin_tsdf - tsdf_value) * weight_tsdf + (orgin_imu_error - imu_error_value) * weight_imu;

                        sum_t_x += tx * weight;
                        sum_t_y += ty * weight;
                        sum_t_z += tz * weight;
                        sum_q_x += qx * weight;
                        sum_q_y += qy * weight;
                        sum_q_z += qz * weight;

                        sum_bias_acc_x += disturb_imu_bias_acc_x * weight;
                        sum_bias_acc_y += disturb_imu_bias_acc_y * weight;
                        sum_bias_acc_z += disturb_imu_bias_acc_z * weight;
                        sum_bias_gyr_x += disturb_imu_bias_gyr_x * weight;
                        sum_bias_gyr_y += disturb_imu_bias_gyr_y * weight;
                        sum_bias_gyr_z += disturb_imu_bias_gyr_z * weight;

                        qx = qx * (double)search_size(3, 0);
                        qy = qy * (double)search_size(4, 0);
                        qz = qz * (double)search_size(5, 0);

                        const double qw = sqrt(1 - qx * qx - qy * qy - qz * qz);

                        sum_q_w += qw * weight;

                        sum_weight_sum += weight;

                        sum_mean_fitness += weight * fitness;
                        ++count_search;
                    }
                    if (count_search == 200)
                    {
                        break;
                    }
                }

                // 更新位姿！
                if (count_search <= 0)
                {

                    *min_tsdf = orgin_fitness; // 当depth被遮挡的时候这个值一定为0，这就会导致后续的所有的搜索都是废的！
                    return false;
                }

                if (iter_times > threshold_usage_imu)
                {
                    imu_model.bias_acc.x() += sum_bias_acc_x * imu_search_size(0, 0) / sum_weight_sum;
                    imu_model.bias_acc.y() += sum_bias_acc_y * imu_search_size(0, 1) / sum_weight_sum;
                    imu_model.bias_acc.z() += sum_bias_acc_z * imu_search_size(0, 2) / sum_weight_sum;

                    imu_model.bias_gyr.x() += sum_bias_gyr_x * imu_search_size(0, 3) / sum_weight_sum;
                    imu_model.bias_gyr.y() += sum_bias_gyr_y * imu_search_size(0, 4) / sum_weight_sum;
                    imu_model.bias_gyr.z() += sum_bias_gyr_z * imu_search_size(0, 5) / sum_weight_sum;
                }

                // std::cout<<"imu_model.bias_gyr:\n"<<imu_model.bias_gyr<<std::endl;
                // std::cout<<"imu_model.bias_acc:\n"<<imu_model.bias_acc<<std::endl;

                mean_transform(0, 0) = sum_t_x;
                mean_transform(1, 0) = sum_t_y;
                mean_transform(2, 0) = sum_t_z;

                mean_transform(3, 0) = sum_q_w;
                mean_transform(4, 0) = sum_q_x;
                mean_transform(5, 0) = sum_q_y;
                mean_transform(6, 0) = sum_q_z;

                // const double weight_sum=sum_weight_sum;
                double mean_fitness = sum_mean_fitness / sum_weight_sum;

                mean_transform = mean_transform / sum_weight_sum;

                mean_transform(0, 0) = mean_transform(0, 0) * (double)search_size(0, 0);
                mean_transform(1, 0) = mean_transform(1, 0) * (double)search_size(1, 0);
                mean_transform(2, 0) = mean_transform(2, 0) * (double)search_size(2, 0);

                double qw = mean_transform(3, 0);
                double qx = mean_transform(4, 0) * search_size(3, 0);
                double qy = mean_transform(5, 0) * search_size(4, 0);
                double qz = mean_transform(6, 0) * search_size(5, 0);
                double lens = 1 / sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
                // 归一化！
                mean_transform(3, 0) = qw * lens;
                mean_transform(4, 0) = qx * lens;
                mean_transform(5, 0) = qy * lens;
                mean_transform(6, 0) = qz * lens;

                *min_tsdf = mean_fitness;
                // std::cout<<"end iteration"<<std::endl;
                return true;
            }

            bool particle_evaluation_hash(const ITMVoxel_d *voxelData, const ITMHashEntry *hashTable, const VolumeData &volume,
                                          const QuaternionData &quaterinons, SearchData &search_data, const Eigen::Matrix3d &rotation_current, const Matf31da &translation_current,
                                          const cv::cuda::GpuMat &gpu_compact_vertex, int compact_size,
                                          const Eigen::Matrix3d &rotation_previous_inv, const Matf31da &translation_previous,
                                          const CameraParameters &cam_params, const int particle_index, const int particle_size, // particle_index对应使用哪种粒子群（不同群的候选位姿和粒子个数不同）particle_size对应三种粒子群个数中的一个，表明该粒子群个数
                                          const Matf61da &search_size, int level, const int level_index,
                                          Eigen::Matrix<double, 7, 1> &mean_transform, float *min_tsdf, IMU_Model &imu_model,
                                          const Eigen::Vector3d incre_translation, const Eigen::Matrix3d incre_rotation,
                                          const Eigen::Matrix3d frame_rotation, int iter_times)

            {
                std::cout.precision(17);
                // std::cout<<"begin iteration"<<std::endl;
                const int cols = cam_params.image_width;
                const int rows = cam_params.image_height;

                dim3 block(BLOCK_SIZE_X * BLOCK_SIZE_Y, 1, 1);
                dim3 grid(1, 1, 1);
                grid.x = static_cast<unsigned int>(std::ceil((float)particle_size / block.y / block.x));

                grid.y = level;                                  // 这个level干嘛的
                int stride = (compact_size - level + 1) / level; // 步长

                search_data.gpu_search_count[particle_index / 20].setTo(0);
                search_data.gpu_search_value[particle_index / 20].setTo(0);
                // 采样候选位姿，得到一个残差
                particle_kernel_tsdf_hash<<<grid, block>>>(voxelData, hashTable, gpu_compact_vertex, compact_size,
                                                           search_data.gpu_search_value[particle_index / 20],
                                                           search_data.gpu_search_count[particle_index / 20],
                                                           rotation_current.cast<float>(), translation_current.cast<float>(),
                                                           rotation_previous_inv.cast<float>(), translation_previous.cast<float>(), quaterinons.q[particle_index], // 粒子群，即候选位姿
                                                           cam_params, volume.voxel_scale, particle_size, cols, rows, search_size,
                                                           stride, level_index);

                cv::Mat search_data_count = search_data.search_count[particle_index / 20];
                cv::Mat search_data_value = search_data.search_value[particle_index / 20];
                search_data.gpu_search_count[particle_index / 20].download(search_data_count);
                search_data.gpu_search_value[particle_index / 20].download(search_data_value);
                std::cout << "pose estimation tsdf: " << (double)search_data_value.ptr<int>(0)[0] / (double)search_data_count.ptr<int>(0)[0] << std::endl;
                if ((double)search_data_count.ptr<int>(0)[0] < 10) // 说明原始图像可用点！
                {
                    std::cout << "no depth tsdf res: " << (double)search_data_value.ptr<int>(0)[0] << std::endl;
                }

                dim3 block2(BLOCK_SIZE_X, 1, 1);
                dim3 grid2(1, 1, 1);
                grid2.x = static_cast<unsigned int>(std::ceil((double)particle_size / block.y));

                Eigen::Matrix3d w_2_imu_rotation = imu_model.imu_2_cam_rotation.transpose() * frame_rotation.transpose(); // 世界->imu的旋转！，是在上一帧的基础下的
                int pst_type = particle_index / 20;                                                                       // 表示是10240 3072 1024中的那个粒子个数
                int new_particle_index = ((particle_index + 1) % 20) + pst_type * 20;                                     // 这是当前粒子数的下一个粒子数索引！

                Matf61da imu_search_size;
                imu_search_size << 0.0005, 0.0005, 0.0005, 0.0001, 0.0001, 0.0001;

                int threshold_usage_imu = 15;
                double weight_imu;
                cv::Mat imu_search_value = search_data.imu_search_value[particle_index / 20];

                if (iter_times < threshold_usage_imu) // iter_times迭代次数
                {
                    weight_imu = 0;
                }
                else
                {
                    // weight_imu=0;
                    weight_imu = 0.1;

                    search_data.gpu_imu_search_value[particle_index / 20].setTo(0);
                    // 这个平移残差和旋转残差是通过随机采样零偏，再积分imu得到 P,Q,再与选定的候选粒子的偏差，即增量位姿做差得到的！
                    particle_evaluation_imu<<<grid2, block2>>>(search_data.gpu_imu_mat[2], //(gpu_imu_mat[2] = 100*7)
                                                               frame_rotation,
                                                               incre_rotation, incre_translation, // 当前相机坐标系下的增量
                                                               imu_model.latest_v, imu_model.g,
                                                               imu_model.bias_acc, imu_model.bias_gyr,
                                                               imu_model.imu_2_cam, w_2_imu_rotation,
                                                               search_data.gpu_imu_search_value[particle_index / 20],
                                                               quaterinons.q[particle_index],
                                                               quaterinons.q[new_particle_index],
                                                               particle_size, // particle_size对应三种粒子群个数中的一个，表明该粒子群个数
                                                               imu_model.v_imu.size(),
                                                               search_size,
                                                               imu_search_size);
                    search_data.gpu_imu_search_value[particle_index / 20].download(imu_search_value);
                }

                cudaDeviceSynchronize();

                clock_t time_2 = clock();

                int count_search = 0;
                const int iter_rows = particle_size;

                double sum_t_x = 0.0;
                double sum_t_y = 0.0;
                double sum_t_z = 0.0;
                double sum_q_x = 0.0;
                double sum_q_y = 0.0;
                double sum_q_z = 0.0;
                double sum_q_w = 0.0;

                double sum_bias_acc_x = 0.0;
                double sum_bias_acc_y = 0.0;
                double sum_bias_acc_z = 0.0;
                double sum_bias_gyr_x = 0.0;
                double sum_bias_gyr_y = 0.0;
                double sum_bias_gyr_z = 0.0;

                double sum_weight_sum = 0.0;
                double sum_mean_fitness = 0.0;

                double weight_tsdf = 1;

                double orgin_tsdf = (double)search_data_value.ptr<int>(0)[0] / (double)search_data_count.ptr<int>(0)[0] * DIVSHORTMAX;
                int orgin_count = search_data_count.ptr<int>(0)[0];

                double orgin_imu_error = (double)imu_search_value.ptr<float>(0)[0];

                double orgin_fitness = orgin_tsdf * weight_tsdf + orgin_imu_error * weight_imu;
                // printf("orgin:%f %f %f %d\n",orgin_fitness,orgin_tsdf,orgin_imu_error,orgin_count);

                for (int i = 1; i < iter_rows; ++i) // 遍历所有粒子！
                {

                    double tsdf_value = (double)search_data_value.ptr<int>(i)[0] / (double)search_data_count.ptr<int>(i)[0] * DIVSHORTMAX;
                    double imu_error_value = (double)imu_search_value.ptr<float>(i)[0];
                    double fitness = tsdf_value * weight_tsdf + imu_error_value * weight_imu;
                    // printf("%f %f %f\n",fitness,tsdf_value,imu_error_value);

                    if (fitness < orgin_fitness && ((search_data_count.ptr<int>(i)[0]) > (orgin_count / 2.0)))
                    {

                        const double tx = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[0];
                        const double ty = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[1];
                        const double tz = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[2];
                        double qx = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[3];
                        double qy = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[4];
                        double qz = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[5];

                        const double disturb_imu_bias_acc_x = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[0];
                        const double disturb_imu_bias_acc_y = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[1];
                        const double disturb_imu_bias_acc_z = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[2];
                        const double disturb_imu_bias_gyr_x = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[3];
                        const double disturb_imu_bias_gyr_y = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[4];
                        const double disturb_imu_bias_gyr_z = (double)quaterinons.q_trans[new_particle_index].ptr<float>(i)[5];

                        const double weight = (orgin_tsdf - tsdf_value) * weight_tsdf + (orgin_imu_error - imu_error_value) * weight_imu;

                        sum_t_x += tx * weight;
                        sum_t_y += ty * weight;
                        sum_t_z += tz * weight;
                        sum_q_x += qx * weight;
                        sum_q_y += qy * weight;
                        sum_q_z += qz * weight;

                        sum_bias_acc_x += disturb_imu_bias_acc_x * weight;
                        sum_bias_acc_y += disturb_imu_bias_acc_y * weight;
                        sum_bias_acc_z += disturb_imu_bias_acc_z * weight;
                        sum_bias_gyr_x += disturb_imu_bias_gyr_x * weight;
                        sum_bias_gyr_y += disturb_imu_bias_gyr_y * weight;
                        sum_bias_gyr_z += disturb_imu_bias_gyr_z * weight;

                        qx = qx * (double)search_size(3, 0);
                        qy = qy * (double)search_size(4, 0);
                        qz = qz * (double)search_size(5, 0);

                        const double qw = sqrt(1 - qx * qx - qy * qy - qz * qz);

                        sum_q_w += qw * weight;

                        sum_weight_sum += weight;

                        sum_mean_fitness += weight * fitness;
                        ++count_search;
                    }
                    if (count_search == 200)
                    {
                        break;
                    }
                }

                // 更新位姿！
                if (count_search <= 0)
                {

                    *min_tsdf = orgin_fitness; // 当depth被遮挡的时候这个值一定为0，这就会导致后续的所有的搜索都是废的！
                    return false;
                }

                if (iter_times > threshold_usage_imu)
                {
                    imu_model.bias_acc.x() += sum_bias_acc_x * imu_search_size(0, 0) / sum_weight_sum;
                    imu_model.bias_acc.y() += sum_bias_acc_y * imu_search_size(0, 1) / sum_weight_sum;
                    imu_model.bias_acc.z() += sum_bias_acc_z * imu_search_size(0, 2) / sum_weight_sum;

                    imu_model.bias_gyr.x() += sum_bias_gyr_x * imu_search_size(0, 3) / sum_weight_sum;
                    imu_model.bias_gyr.y() += sum_bias_gyr_y * imu_search_size(0, 4) / sum_weight_sum;
                    imu_model.bias_gyr.z() += sum_bias_gyr_z * imu_search_size(0, 5) / sum_weight_sum;
                }

                mean_transform(0, 0) = sum_t_x;
                mean_transform(1, 0) = sum_t_y;
                mean_transform(2, 0) = sum_t_z;

                mean_transform(3, 0) = sum_q_w;
                mean_transform(4, 0) = sum_q_x;
                mean_transform(5, 0) = sum_q_y;
                mean_transform(6, 0) = sum_q_z;

                // const double weight_sum=sum_weight_sum;
                double mean_fitness = sum_mean_fitness / sum_weight_sum;

                mean_transform = mean_transform / sum_weight_sum;

                mean_transform(0, 0) = mean_transform(0, 0) * (double)search_size(0, 0);
                mean_transform(1, 0) = mean_transform(1, 0) * (double)search_size(1, 0);
                mean_transform(2, 0) = mean_transform(2, 0) * (double)search_size(2, 0);

                double qw = mean_transform(3, 0);
                double qx = mean_transform(4, 0) * search_size(3, 0);
                double qy = mean_transform(5, 0) * search_size(4, 0);
                double qz = mean_transform(6, 0) * search_size(5, 0);
                double lens = 1 / sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
                // 归一化！
                mean_transform(3, 0) = qw * lens;
                mean_transform(4, 0) = qx * lens;
                mean_transform(5, 0) = qy * lens;
                mean_transform(6, 0) = qz * lens;

                *min_tsdf = mean_fitness;
                // std::cout<<"end iteration"<<std::endl;
                return true;
            }
        }
    }
}