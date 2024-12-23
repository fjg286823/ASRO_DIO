#include "include/common.h"

#define BLOCK_SIZE_X 32
#define BLOCK_SIZE_Y 32

using Matf31da = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matf61da = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
using Matf31fa = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
using Matf61fa = Eigen::Matrix<float, 6, 1, Eigen::DontAlign>;
using Matf51da = Eigen::Matrix<double, 5, 1, Eigen::DontAlign>;
using Matf51ia = Eigen::Matrix<int, 5, 1, Eigen::DontAlign>;

namespace rosefusion
{
    namespace internal
    {
        namespace cuda
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
                Eigen::Vector3d
                intergration_time_frame(const Eigen::Vector3d g,
                                        const Eigen::Matrix3d w_2_imu_rotation, Eigen::Vector3d &imu_sample_v,
                                        const PtrStep<float> imu_data, const int n_imu,
                                        Eigen::Vector3d bias_acc,
                                        Eigen::Vector3d bias_gyr)
            {

                Eigen::Vector3d latest_P(0, 0, 0);
                Eigen::Vector3d imu_g = w_2_imu_rotation * g;
                // Eigen::Vector3d imu_v=imu_sample_v;

                double latest_time = imu_data.ptr(0)[0];
                Eigen::Vector3d latest_acc(imu_data.ptr(0)[1], imu_data.ptr(0)[2], imu_data.ptr(0)[3]);
                Eigen::Vector3d latest_gyr(imu_data.ptr(0)[4], imu_data.ptr(0)[5], imu_data.ptr(0)[6]);
                latest_acc -= bias_acc;
                latest_gyr -= bias_gyr;
                double4 latest_Q = make_double4(0, 0, 0, 1);

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
                    // acc=acc-bias_acc;
                    // gyr=gyr-bias_gyr;
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

                return latest_P;
            }
            // 根据随机采样的v和g重新积分两帧之间的imu数据得到平移，再与前面的粒子滤波得到的相对平移作差作为残差，残差总共利用了3帧来计算
            __global__ void gravity_velocity_kernel(PtrStep<float> search_value,

                                                    const Matf51ia imu_data_size,

                                                    const PtrStep<float> imu_info,

                                                    const PtrStep<float> imu_data_0,
                                                    const PtrStep<float> imu_data_1,
                                                    const PtrStep<float> imu_data_2,
                                                    // const PtrStep<float> imu_data_3,
                                                    // const PtrStep<float> imu_data_4,

                                                    const Matf31da init_v, const Matf31da init_g,
                                                    const Eigen::Vector3d bias_acc,
                                                    const Eigen::Vector3d bias_gyr,
                                                    const PtrStep<float> quaternion_trans,
                                                    const int particle_size,
                                                    const Matf61da search_size)

            {
                const int p = blockIdx.x * blockDim.x + threadIdx.x; // 粒子索引

                // printf("%d",p);
                // return;
                if (p >= particle_size)
                {
                    return;
                }
                // if (p!=0){
                //     return;
                // }

                // world coordinate
                const double disturb_v_x = (double)quaternion_trans.ptr(p)[0] * search_size(0, 0);
                const double disturb_v_y = (double)quaternion_trans.ptr(p)[1] * search_size(1, 0);
                const double disturb_v_z = (double)quaternion_trans.ptr(p)[2] * search_size(2, 0);
                const Eigen::Vector3d disturb_v(disturb_v_x, disturb_v_y, disturb_v_z);

                const double qx = (double)quaternion_trans.ptr(p)[3] * search_size(3, 0);
                const double qy = (double)quaternion_trans.ptr(p)[4] * search_size(4, 0);
                const double qz = (double)quaternion_trans.ptr(p)[5] * search_size(5, 0);
                const double qw = sqrt(1 - qx * qx - qy * qy - qz * qz);

                Eigen::Matrix3d disturb_g;

                disturb_g << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
                    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
                    2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy);

                Eigen::Vector3d imu_sample_v = init_v.cast<double>() + disturb_v.cast<double>();
                // printf("imu v: %f %f %f\n",imu_sample_v.x(),imu_sample_v.y(),imu_sample_v.z());
                Eigen::Vector3d sample_g = disturb_g.cast<double>() * init_g.cast<double>(); // 世界坐标系下的

                // Eigen::Vector3d intergration_time_frame(const Eigen::Vector3d g,
                //     const Eigen::Matrix3d w_2_imu_rotation, Eigen::Vector3d &imu_sample_v,
                //     const PtrStep<float> imu_data, const int n_imu){

                float sum_error = 0;
                int i = 0;
                // printf("imu 0: %d\n",imu_data_size(i,0));

                Eigen::Vector3d estimation_translation;
                Eigen::Matrix3d w_2_imu_rotation;
                Eigen::Vector3d t;

                if (imu_data_size(i, 0) > 0)
                {
                    estimation_translation << imu_info.ptr(0 + i * 4)[0], imu_info.ptr(0 + i * 4)[1], imu_info.ptr(0 + i * 4)[2];

                    // Eigen::Matrix3d w_2_imu_rotation;
                    w_2_imu_rotation << imu_info.ptr(1 + i * 4)[0], imu_info.ptr(1 + i * 4)[1], imu_info.ptr(1 + i * 4)[2],
                        imu_info.ptr(2 + i * 4)[0], imu_info.ptr(2 + i * 4)[1], imu_info.ptr(2 + i * 4)[2],
                        imu_info.ptr(3 + i * 4)[0], imu_info.ptr(3 + i * 4)[1], imu_info.ptr(3 + i * 4)[2];
                    // 此处的g和v是采样之后的结果，根据新的g,v积分得到的结果
                    t = intergration_time_frame(sample_g, w_2_imu_rotation, imu_sample_v, imu_data_0, imu_data_size(i, 0), bias_acc, bias_gyr);
                    sum_error += (estimation_translation - t).norm();
                    // printf("estimation_translation:%f %f %f", estimation_translation.x(),estimation_translation.y(),estimation_translation.z());

                    // printf("estimation_translation:%f %f %f", estimation_translation.x(),estimation_translation.y(),estimation_translation.z());
                    // printf("t:%f %f %f", t.x(),t.y(),t.z());

                    i++;
                }

                if (imu_data_size(i, 0) > 0)
                {
                    // Eigen::Vector3d estimation_translation;
                    estimation_translation << imu_info.ptr(0 + i * 4)[0], imu_info.ptr(0 + i * 4)[1], imu_info.ptr(0 + i * 4)[2];

                    // Eigen::Matrix3d w_2_imu_rotation;
                    w_2_imu_rotation << imu_info.ptr(1 + i * 4)[0], imu_info.ptr(1 + i * 4)[1], imu_info.ptr(1 + i * 4)[2],
                        imu_info.ptr(2 + i * 4)[0], imu_info.ptr(2 + i * 4)[1], imu_info.ptr(2 + i * 4)[2],
                        imu_info.ptr(3 + i * 4)[0], imu_info.ptr(3 + i * 4)[1], imu_info.ptr(3 + i * 4)[2];

                    t = intergration_time_frame(sample_g, w_2_imu_rotation, imu_sample_v, imu_data_1, imu_data_size(i, 0), bias_acc, bias_gyr);
                    sum_error += (estimation_translation - t).norm();
                    // printf("estimation_translation:%f %f %f", estimation_translation.x(),estimation_translation.y(),estimation_translation.z());

                    i++;
                }

                if (imu_data_size(i, 0) > 0)
                {
                    // Eigen::Vector3d estimation_translation;
                    estimation_translation << imu_info.ptr(0 + i * 4)[0], imu_info.ptr(0 + i * 4)[1], imu_info.ptr(0 + i * 4)[2];

                    w_2_imu_rotation << imu_info.ptr(1 + i * 4)[0], imu_info.ptr(1 + i * 4)[1], imu_info.ptr(1 + i * 4)[2],
                        imu_info.ptr(2 + i * 4)[0], imu_info.ptr(2 + i * 4)[1], imu_info.ptr(2 + i * 4)[2],
                        imu_info.ptr(3 + i * 4)[0], imu_info.ptr(3 + i * 4)[1], imu_info.ptr(3 + i * 4)[2];

                    t = intergration_time_frame(sample_g, w_2_imu_rotation, imu_sample_v, imu_data_2, imu_data_size(i, 0), bias_acc, bias_gyr);
                    sum_error += (estimation_translation - t).norm();
                    // printf("estimation_translation:%f %f %f", estimation_translation.x(),estimation_translation.y(),estimation_translation.z());

                    i++;
                }

                search_value.ptr(p)[0] = sum_error;
            }
            // 采样bias执行imu积分基于之前估计好的相对平移作差得到残差，残差是连续3帧的和
            __global__ void bias_kernel(PtrStep<float> search_value,

                                        const Matf51ia imu_data_size,

                                        const PtrStep<float> imu_info,

                                        const PtrStep<float> imu_data_0,
                                        const PtrStep<float> imu_data_1,
                                        const PtrStep<float> imu_data_2,
                                        // const PtrStep<float> imu_data_3,
                                        // const PtrStep<float> imu_data_4,

                                        const Matf31da init_v, const Matf31da init_g,
                                        const Eigen::Vector3d bias_acc,
                                        const Eigen::Vector3d bias_gyr,
                                        const PtrStep<float> quaternion_trans,
                                        const int particle_size,
                                        const Matf61da search_size_imu)

            {
                const int p = blockIdx.x * blockDim.x + threadIdx.x;

                if (p >= particle_size)
                {
                    return;
                }

                // if (p!=0){
                //     return;
                // }
                Eigen::Vector3d imu_sample_v = init_v.cast<double>();
                Eigen::Vector3d sample_g = init_g.cast<double>();

                const double bias_acc_x = bias_acc.x() + (double)quaternion_trans.ptr(p)[0] * search_size_imu(0, 0);
                const double bias_acc_y = bias_acc.y() + (double)quaternion_trans.ptr(p)[1] * search_size_imu(1, 0);
                const double bias_acc_z = bias_acc.z() + (double)quaternion_trans.ptr(p)[2] * search_size_imu(2, 0);
                const double bias_gyr_x = bias_gyr.x() + (double)quaternion_trans.ptr(p)[3] * search_size_imu(3, 0);
                const double bias_gyr_y = bias_gyr.y() + (double)quaternion_trans.ptr(p)[4] * search_size_imu(4, 0);
                const double bias_gyr_z = bias_gyr.z() + (double)quaternion_trans.ptr(p)[5] * search_size_imu(5, 0);

                Eigen::Vector3d sample_bias_acc(bias_acc_x, bias_acc_y, bias_acc_z);
                Eigen::Vector3d sample_bias_gyr(bias_gyr_x, bias_gyr_y, bias_gyr_z);
                // Eigen::Vector3d sample_bias_acc(bias_acc.x(),bias_acc.y(),bias_acc.z());
                // Eigen::Vector3d sample_bias_gyr(bias_gyr.x(),bias_gyr.y(),bias_gyr.z());

                float sum_error = 0;
                int i = 0;
                // printf("imu 0: %d\n",imu_data_size(i,0));

                Eigen::Vector3d estimation_translation;
                Eigen::Matrix3d w_2_imu_rotation;
                Eigen::Vector3d t;

                if (imu_data_size(i, 0) > 0)
                {
                    estimation_translation << imu_info.ptr(0 + i * 4)[0], imu_info.ptr(0 + i * 4)[1], imu_info.ptr(0 + i * 4)[2];

                    // Eigen::Matrix3d w_2_imu_rotation;
                    w_2_imu_rotation << imu_info.ptr(1 + i * 4)[0], imu_info.ptr(1 + i * 4)[1], imu_info.ptr(1 + i * 4)[2],
                        imu_info.ptr(2 + i * 4)[0], imu_info.ptr(2 + i * 4)[1], imu_info.ptr(2 + i * 4)[2],
                        imu_info.ptr(3 + i * 4)[0], imu_info.ptr(3 + i * 4)[1], imu_info.ptr(3 + i * 4)[2];

                    t = intergration_time_frame(sample_g, w_2_imu_rotation, imu_sample_v, imu_data_0, imu_data_size(i, 0), sample_bias_acc, sample_bias_gyr);
                    sum_error += (estimation_translation - t).norm();
                    // printf("estimation_translation:%f %f %f", estimation_translation.x(),estimation_translation.y(),estimation_translation.z());
                    // printf("t:%f %f %f", t.x(),t.y(),t.z());

                    i++;
                }

                if (imu_data_size(i, 0) > 0)
                {
                    // Eigen::Vector3d estimation_translation;
                    estimation_translation << imu_info.ptr(0 + i * 4)[0], imu_info.ptr(0 + i * 4)[1], imu_info.ptr(0 + i * 4)[2];

                    // Eigen::Matrix3d w_2_imu_rotation;
                    w_2_imu_rotation << imu_info.ptr(1 + i * 4)[0], imu_info.ptr(1 + i * 4)[1], imu_info.ptr(1 + i * 4)[2],
                        imu_info.ptr(2 + i * 4)[0], imu_info.ptr(2 + i * 4)[1], imu_info.ptr(2 + i * 4)[2],
                        imu_info.ptr(3 + i * 4)[0], imu_info.ptr(3 + i * 4)[1], imu_info.ptr(3 + i * 4)[2];

                    t = intergration_time_frame(sample_g, w_2_imu_rotation, imu_sample_v, imu_data_1, imu_data_size(i, 0), sample_bias_acc, sample_bias_gyr);
                    sum_error += (estimation_translation - t).norm();
                    // printf("estimation_translation:%f %f %f", estimation_translation.x(),estimation_translation.y(),estimation_translation.z());

                    i++;
                }

                if (imu_data_size(i, 0) > 0)
                {
                    // Eigen::Vector3d estimation_translation;
                    estimation_translation << imu_info.ptr(0 + i * 4)[0], imu_info.ptr(0 + i * 4)[1], imu_info.ptr(0 + i * 4)[2];

                    w_2_imu_rotation << imu_info.ptr(1 + i * 4)[0], imu_info.ptr(1 + i * 4)[1], imu_info.ptr(1 + i * 4)[2],
                        imu_info.ptr(2 + i * 4)[0], imu_info.ptr(2 + i * 4)[1], imu_info.ptr(2 + i * 4)[2],
                        imu_info.ptr(3 + i * 4)[0], imu_info.ptr(3 + i * 4)[1], imu_info.ptr(3 + i * 4)[2];

                    t = intergration_time_frame(sample_g, w_2_imu_rotation, imu_sample_v, imu_data_2, imu_data_size(i, 0), sample_bias_acc, sample_bias_gyr);
                    sum_error += (estimation_translation - t).norm();
                    // printf("estimation_translation:%f %f %f", estimation_translation.x(),estimation_translation.y(),estimation_translation.z());

                    i++;
                }

                search_value.ptr(p)[0] = sum_error;
            }

            bool bias_particle_evaluation(const QuaternionData &quaterinons,
                                          SearchData &search_data,
                                          const Matf51ia imu_data_size,
                                          const Eigen::Vector3d &init_v,
                                          const Eigen::Vector3d &init_g,
                                          IMU_Model &imu_model,
                                          const int particle_index, const int particle_size,
                                          const Matf61da &imu_search_size,
                                          double *min_error)
            {

                // return false;

                dim3 block(BLOCK_SIZE_X, 1, 1);
                dim3 grid(1, 1, 1);
                grid.x = static_cast<unsigned int>(std::ceil((double)particle_size / block.y));

                // std::cout<<"init_v:\n"<<init_v<<std::endl;
                // std::cout<<"init_g:\n"<<init_g<<std::endl;

                search_data.gpu_imu_search_value[particle_index / 20].setTo(0);
                bias_kernel<<<grid, block>>>(search_data.gpu_imu_search_value[particle_index / 20],

                                             imu_data_size,
                                             search_data.gpu_imu_info,

                                             search_data.gpu_imu_mat[0],
                                             search_data.gpu_imu_mat[1],
                                             search_data.gpu_imu_mat[2],
                                             // search_data.gpu_imu_mat[3],
                                             // search_data.gpu_imu_mat[4],

                                             init_v.cast<double>(), init_g.cast<double>(),
                                             // bias_acc,bias_gyr,
                                             imu_model.bias_acc,
                                             imu_model.bias_gyr,
                                             quaterinons.q[particle_index],
                                             particle_size, imu_search_size);

                cv::Mat imu_search_value = search_data.imu_search_value[particle_index / 20];
                search_data.gpu_imu_search_value[particle_index / 20].download(imu_search_value);
                const int iter_rows = particle_size;

                double sum_bias_acc_x = 0.0;
                double sum_bias_acc_y = 0.0;
                double sum_bias_acc_z = 0.0;
                double sum_bias_gyr_x = 0.0;
                double sum_bias_gyr_y = 0.0;
                double sum_bias_gyr_z = 0.0;
                double sum_weight_sum = 0.0;
                double sum_error = 0.0;
                int count_search = 0;
                double orgin_imu_error = (double)imu_search_value.ptr<float>(0)[0];
                // std::cout<<"orgin_imu_error"<<orgin_imu_error<<std::endl;
                for (int i = 1; i < iter_rows; ++i)
                {
                    double imu_error_value = (double)imu_search_value.ptr<float>(i)[0];
                    // std::cout<<"imu_error"<<imu_error_value<<std::endl;

                    if (imu_error_value < orgin_imu_error)
                    {

                        const double disturb_imu_bias_acc_x = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[0];
                        const double disturb_imu_bias_acc_y = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[1];
                        const double disturb_imu_bias_acc_z = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[2];
                        const double disturb_imu_bias_gyr_x = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[3];
                        const double disturb_imu_bias_gyr_y = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[4];
                        const double disturb_imu_bias_gyr_z = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[5];
                        const double weight = (orgin_imu_error - imu_error_value);

                        sum_bias_acc_x += disturb_imu_bias_acc_x * weight;
                        sum_bias_acc_y += disturb_imu_bias_acc_y * weight;
                        sum_bias_acc_z += disturb_imu_bias_acc_z * weight;
                        sum_bias_gyr_x += disturb_imu_bias_gyr_x * weight;
                        sum_bias_gyr_y += disturb_imu_bias_gyr_y * weight;
                        sum_bias_gyr_z += disturb_imu_bias_gyr_z * weight;

                        sum_weight_sum += weight;
                        sum_error += weight * imu_error_value;
                        ++count_search;
                    }
                    if (count_search == 200)
                    {
                        break;
                    }
                }
                // exit(0);
                if (count_search <= 0)
                {
                    *min_error = orgin_imu_error;
                    return false;
                }

                imu_model.bias_acc.x() += sum_bias_acc_x * imu_search_size(0, 0) / sum_weight_sum;
                imu_model.bias_acc.y() += sum_bias_acc_y * imu_search_size(0, 1) / sum_weight_sum;
                imu_model.bias_acc.z() += sum_bias_acc_z * imu_search_size(0, 2) / sum_weight_sum;

                imu_model.bias_gyr.x() += sum_bias_gyr_x * imu_search_size(0, 3) / sum_weight_sum;
                imu_model.bias_gyr.y() += sum_bias_gyr_y * imu_search_size(0, 4) / sum_weight_sum;
                imu_model.bias_gyr.z() += sum_bias_gyr_z * imu_search_size(0, 5) / sum_weight_sum;

                // std::cout<<"imu_model.bias_gyr:\n"<<imu_model.bias_gyr<<std::endl;
                // std::cout<<"imu_model.bias_acc:\n"<<imu_model.bias_acc<<std::endl;

                *min_error = sum_error / sum_weight_sum;

                return true;
            }
            // 粒子滤波得到v,和重力对应的旋转矩阵！
            bool g_particle_evaluation(const QuaternionData &quaterinons,
                                       SearchData &search_data,

                                       const Matf51ia imu_data_size,

                                       const Eigen::Vector3d &init_v,
                                       const Eigen::Vector3d &init_g,

                                       // const Eigen::Vector3d bias_acc,
                                       // const Eigen::Vector3d bias_gyr,
                                       IMU_Model &imu_model,
                                       const int particle_index, const int particle_size, // 粒子个数
                                       const Matf61da &search_size,
                                       Eigen::Matrix<double, 7, 1> &mean_transform,
                                       double *min_error)
            {
                std::cout.precision(17);

                dim3 block(BLOCK_SIZE_X, 1, 1);
                dim3 grid(1, 1, 1);
                grid.x = static_cast<unsigned int>(std::ceil((double)particle_size / block.y));

                search_data.gpu_search_value[particle_index / 20].setTo(0);
                // 根据随机采样的v和g（注意这里的g其实是采样旋转，来旋转g得到正确的重力向量！）重新积分两帧之间的imu数据得到平移，再与前面的粒子滤波得到的相对平移作差作为残差，残差总共利用了3帧来计算
                gravity_velocity_kernel<<<grid, block>>>(search_data.gpu_search_value[particle_index / 20],

                                                         imu_data_size,
                                                         search_data.gpu_imu_info,

                                                         search_data.gpu_imu_mat[0],
                                                         search_data.gpu_imu_mat[1],
                                                         search_data.gpu_imu_mat[2],
                                                         // search_data.gpu_imu_mat[3],
                                                         // search_data.gpu_imu_mat[4],

                                                         init_v.cast<double>(), init_g.cast<double>(),
                                                         // bias_acc,bias_gyr,
                                                         imu_model.bias_acc,
                                                         imu_model.bias_gyr,
                                                         quaterinons.q[particle_index],
                                                         particle_size, search_size);

                // cudaError_t cudaerr = cudaDeviceSynchronize();
                // if (cudaerr != cudaSuccess)
                //     printf("kernel launch failed with error \"%s\".\n",
                //             cudaGetErrorString(cudaerr));

                // cudaError_t err = cudaGetLastError();
                // if (err != cudaSuccess)
                //     printf("Error: %s\n", cudaGetErrorString(err));

                cv::Mat search_data_value = search_data.search_value[particle_index / 20];
                search_data.gpu_search_value[particle_index / 20].download(search_data_value);
                cudaDeviceSynchronize();

                double orgin_error = (double)search_data_value.ptr<float>(0)[0];

                printf("orgin_error: %f\n", orgin_error);

                clock_t time_2 = clock();

                int count_search = 0.0;
                const int iter_rows = particle_size;

                double sum_t_x = 0.0;
                double sum_t_y = 0.0;
                double sum_t_z = 0.0;
                double sum_q_x = 0.0;
                double sum_q_y = 0.0;
                double sum_q_z = 0.0;
                double sum_q_w = 0.0;
                double sum_weight_sum = 0.0;
                double sum_mean_error = 0.0;

                for (int i = 1; i < iter_rows; ++i)
                {

                    double error_value = (double)search_data_value.ptr<float>(i)[0];
                    // printf("%f, ",error_value);
                    if (error_value < orgin_error)
                    {

                        const double tx = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[0];
                        const double ty = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[1];
                        const double tz = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[2];
                        double qx = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[3];
                        double qy = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[4];
                        double qz = (double)quaterinons.q_trans[particle_index].ptr<float>(i)[5];

                        const double weight = orgin_error - error_value;

                        sum_t_x += tx * weight;
                        sum_t_y += ty * weight;
                        sum_t_z += tz * weight;

                        sum_q_x += qx * weight;
                        sum_q_y += qy * weight;
                        sum_q_z += qz * weight;

                        qx = qx * (double)search_size(3, 0);
                        qy = qy * (double)search_size(4, 0);
                        qz = qz * (double)search_size(5, 0);

                        const double qw = sqrt(1 - qx * qx - qy * qy - qz * qz);

                        sum_q_w += qw * weight;

                        sum_weight_sum += weight;

                        sum_mean_error += weight * error_value;
                        ++count_search;
                    }
                    if (count_search == 200)
                    {
                        break;
                    }
                }

                mean_transform(0, 0) = sum_t_x;
                mean_transform(1, 0) = sum_t_y;
                mean_transform(2, 0) = sum_t_z;
                mean_transform(3, 0) = sum_q_w;
                mean_transform(4, 0) = sum_q_x;
                mean_transform(5, 0) = sum_q_y;
                mean_transform(6, 0) = sum_q_z;
                const double weight_sum = sum_weight_sum;
                double mean_error = sum_mean_error;

                if (count_search <= 0)
                {

                    *min_error = orgin_error;
                    return false;
                }

                mean_transform = mean_transform / weight_sum;
                mean_error = mean_error / weight_sum;

                mean_transform(0, 0) = mean_transform(0, 0) * (double)search_size(0, 0);
                mean_transform(1, 0) = mean_transform(1, 0) * (double)search_size(1, 0);
                mean_transform(2, 0) = mean_transform(2, 0) * (double)search_size(2, 0);

                double qw = mean_transform(3, 0);
                double qx = mean_transform(4, 0) * search_size(3, 0);
                double qy = mean_transform(5, 0) * search_size(4, 0);
                double qz = mean_transform(6, 0) * search_size(5, 0);
                double lens = 1 / sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

                mean_transform(3, 0) = qw * lens;
                mean_transform(4, 0) = qx * lens;
                mean_transform(5, 0) = qy * lens;
                mean_transform(6, 0) = qz * lens;

                *min_error = mean_error;
                // printf("mean_error: %f\n",*min_error);
                // exit(0);
                return true;
            }

        }
    }
}