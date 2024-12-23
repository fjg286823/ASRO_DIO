#include "include/common.h"
#include <ctime>

using cv::cuda::GpuMat;

namespace rosefusion
{
    namespace internal
    {
        namespace cuda
        {

            __global__ void kernel_compute_vertex_map(const PtrStepSz<float> depth_map, PtrStep<float3> vertex_map,
                                                      const float depth_cutoff, const CameraParameters cam_params)
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                if (x >= depth_map.cols || y >= depth_map.rows)
                    return;

                float depth_value = depth_map.ptr(y)[x];
                // printf("%f", depth_value)
                if (depth_value > depth_cutoff)
                {
                    depth_value = 0.f;
                }

                Vec3fda vertex(
                    (x - cam_params.principal_x) * depth_value / cam_params.focal_x,
                    (y - cam_params.principal_y) * depth_value / cam_params.focal_y,
                    depth_value);
                vertex_map.ptr(y)[x] = make_float3(vertex.x(), vertex.y(), vertex.z());
                // printf("point: %f,%f,%f\n",vertex.x(),vertex.y(),vertex.z());
            }

            __global__ void kernel_compute_normal_map(const PtrStepSz<float3> vertex_map, PtrStep<float3> normal_map)
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                if (x < 1 || x >= vertex_map.cols - 1 || y < 1 || y >= vertex_map.rows - 1)
                    return;

                const Vec3fda left(&vertex_map.ptr(y)[x - 1].x);
                const Vec3fda right(&vertex_map.ptr(y)[x + 1].x);
                const Vec3fda upper(&vertex_map.ptr(y - 1)[x].x);
                const Vec3fda lower(&vertex_map.ptr(y + 1)[x].x);
                const Vec3fda center(&vertex_map.ptr(y)[x].x);

                Vec3fda normal;

                if (center.z() == 0 || left.z() == 0 || right.z() == 0 || upper.z() == 0 || lower.z() == 0)
                    normal = Vec3fda(0.f, 0.f, 0.f);
                else
                {
                    Vec3fda hor(left.x() - right.x(), left.y() - right.y(), left.z() - right.z());
                    Vec3fda ver(upper.x() - lower.x(), upper.y() - lower.y(), upper.z() - lower.z());

                    normal = hor.cross(ver);
                    normal.normalize();

                    if (normal.z() > 0)
                        normal *= -1;
                }

                normal_map.ptr(y)[x] = make_float3(normal.x(), normal.y(), normal.z());
                // if (y==1){
                //  printf("%f %f %f %d %d==",normal_map.ptr(y)[x].x,normal_map.ptr(y)[x].y, normal_map.ptr(y)[x].z, y ,x);
                // }
            }

            __global__ // 1024个线程，每个线程处理lenth个像素点
                void
                compute_block_counts(const PtrStep<float3> normal_map, const int length, PtrStep<int> count_block,
                                     const int cols, const int rows)
            {
                const int threadid = (blockIdx.x * blockDim.x + threadIdx.x);
                const int scan_start = threadid * length;
                // if (x!=0){
                //     return;
                // }

                for (int i = 0; i < length; i += 4)
                {
                    const int pi = (scan_start + i) / cols;
                    const int pj = (scan_start + i) - pi * cols; // 得到行列坐标
                    // printf("%d %d %d\n",(x+i), pi, pj);
                    // if (pi%2!=1 || pj%2!=1){
                    //     continue;
                    // }
                    if ((normal_map.ptr(pi)[pj].x == 0 && normal_map.ptr(pi)[pj].y == 0 && normal_map.ptr(pi)[pj].z == 0))
                    {
                        // printf("%f %f %f %d %d\n",normal_map.ptr(pi)[pj].x,normal_map.ptr(pi)[pj].y, normal_map.ptr(pi)[pj].z, pi ,pj);

                        continue;
                    }
                    count_block.ptr(threadid)[0] = count_block.ptr(threadid)[0] + 1;
                }
            }

            __global__ void compact_array(const PtrStep<float3> vertex_map, const PtrStep<float3> normal_map,
                                          const int length, const PtrStep<int> count_block, PtrStep<float3> gpu_compact_vertex,
                                          const int cols, const int rows)
            {
                const int threadid = (blockIdx.x * blockDim.x + threadIdx.x);
                const int scan_start = threadid * length;
                int index_count = 1;
                // int campact_start=0;
                // if (threadid!=0){
                //     campact_start=count_block[threadid-1];
                // }
                for (int i = 0; i < length; i += 4)
                {
                    const int pi = (scan_start + i) / cols;
                    const int pj = (scan_start + i) - pi * cols;
                    // if (pi%2!=1 || pj%2!=1){
                    //     continue;
                    // }
                    if ((normal_map.ptr(pi)[pj].x != 0 || normal_map.ptr(pi)[pj].y != 0 || normal_map.ptr(pi)[pj].z != 0))
                    {
                        int compact_index = count_block[threadid] - index_count;

                        if (gpu_compact_vertex.ptr(compact_index)[0].x != 0)
                        {
                            printf("%d %d %d /", pi, pj, compact_index);
                        }

                        gpu_compact_vertex.ptr(compact_index)[0].x = vertex_map.ptr(pi)[pj].x;
                        gpu_compact_vertex.ptr(compact_index)[0].y = vertex_map.ptr(pi)[pj].y;
                        gpu_compact_vertex.ptr(compact_index)[0].z = vertex_map.ptr(pi)[pj].z;

                        index_count++;
                    }
                }
            }
            // 用于高效从稀疏点云中得到稠密点云
            void compact_vertex(FrameData &frame_data, const CameraParameters &camera_params)
            {
                clock_t time_stt = clock();

                dim3 threads(1024, 1);
                dim3 blocks(1, 1);
                int length = camera_params.image_width * camera_params.image_height / threads.x;

                frame_data.gpu_block_count.setTo(0);
                frame_data.gpu_compact_vertex.setTo(0);
                std::cout << "length: " << length << std::endl; // 将图像划分成1024个区域，然后计算每个区域的有效法线点个数
                compute_block_counts<<<blocks, threads>>>(frame_data.normal_map, length, frame_data.gpu_block_count, camera_params.image_width, camera_params.image_height);

                frame_data.gpu_block_count.download(frame_data.block_count);
                for (int i = 1; i < 1024; i++) // 累积所有区域有效法线点个数，这样最后一个块就是有效点个数
                {
                    // std::cout<<frame_data.block_count.ptr<int>(i)[0]<<"   ";
                    frame_data.block_count.ptr<int>(i)[0] = frame_data.block_count.ptr<int>(i)[0] + frame_data.block_count.ptr<int>(i - 1)[0];
                    // std::cout<<frame_data.block_count.ptr<int>(i)[0]<<std::endl;
                }
                // frame_data.block_count.ptr<int>(1)[0]=frame_data.block_count.ptr<int>(0)[0];
                // frame_data.block_count.ptr<int>(0)[0]=0;

                frame_data.gpu_block_count.upload(frame_data.block_count);

                compact_array<<<blocks, threads>>>(frame_data.vertex_map, frame_data.normal_map,
                                                   length, frame_data.gpu_block_count, frame_data.gpu_compact_vertex,
                                                   camera_params.image_width, camera_params.image_height);

                frame_data.compact_size = frame_data.block_count.ptr<int>(1023)[0]; // 有效点个数（但其实也是降采样后的结果）

                std::cout << "compact time =" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
                // // // write image ++++++++++++++++++++++++++++++++
                // frame_data.gpu_compact_vertex.download(frame_data.compact_vertex);

                // std::ofstream file_out { "/home/jiazhao/code/rosefusion-imu-cd/result/compact_vertex.ply" };
                // if (!file_out.is_open())
                //     return;

                // file_out << "ply" << std::endl;
                // file_out << "format ascii 1.0" << std::endl;
                // // file_out << "element vertex " << frame_data.block_count.ptr<int>(1023)[0] << std::endl;
                // file_out << "element vertex " << frame_data.compact_size << std::endl;

                // file_out << "property float x" << std::endl;
                // file_out << "property float y" << std::endl;
                // file_out << "property float z" << std::endl;

                // file_out << "end_header" << std::endl;

                // FrameData& frame_data, const CameraParameters& camera_params
                // for (int i=0 ; i <frame_data.compact_size; i++){

                //     float3 vertex =frame_data.compact_vertex.ptr<float3>(i)[0];

                //     file_out << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;

                // }
            }

            void compute_vertex_map(const GpuMat &depth_map, GpuMat &vertex_map, const float depth_cutoff,
                                    const CameraParameters cam_params)
            {
                dim3 threads(32, 32);
                dim3 blocks((depth_map.cols + threads.x - 1) / threads.x, (depth_map.rows + threads.y - 1) / threads.y);

                kernel_compute_vertex_map<<<blocks, threads>>>(depth_map, vertex_map, depth_cutoff, cam_params);

                cudaDeviceSynchronize();
            }

            void compute_normal_map(const GpuMat &vertex_map, GpuMat &normal_map)
            {
                dim3 threads(32, 32);
                dim3 blocks((vertex_map.cols + threads.x - 1) / threads.x,
                            (vertex_map.rows + threads.y - 1) / threads.y);

                kernel_compute_normal_map<<<blocks, threads>>>(vertex_map, normal_map);

                cudaDeviceSynchronize();
            }

        }
    }
}