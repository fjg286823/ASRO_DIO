// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <opencv2/core/cuda.hpp>
#include <Eigen/Dense>
#include <math.h>
#include "ITMRepresentationAccess.h"

struct AllocationTempData {
    int noAllocatedVoxelEntries;
    int noAllocatedExcessEntries;
    int noVisibleEntries;
};


template<typename T>
__device__ inline int computePrefixSum_device(uint element, T *sum, int localSize, int localId) {
    __shared__ uint prefixBuffer[16 * 16];
    __shared__ uint groupOffset;

    prefixBuffer[localId] = element;
    __syncthreads();

    int s1, s2;
    for (s1 = 1, s2 = 1; s1 < localSize; s1 <<= 1) {
        s2 |= s1;
        if ((localId & s2) == s2) prefixBuffer[localId] += prefixBuffer[localId - s1];
        __syncthreads();
    }

    for (s1 >>= 2, s2 >>= 1; s1 >= 1; s1 >>= 1, s2 >>= 1) {
        if (localId != localSize - 1 && (localId & s2) == s2) prefixBuffer[localId + s1] += prefixBuffer[localId];
        __syncthreads();
    }

    if (localId == 0 && prefixBuffer[localSize - 1] > 0) groupOffset = atomicAdd(sum, prefixBuffer[localSize - 1]);
    __syncthreads();

    int offset;
    if (localId == 0) {
        if (prefixBuffer[localId] == 0) offset = -1;
        else offset = groupOffset;
    } else {
        if (prefixBuffer[localId] == prefixBuffer[localId - 1]) offset = -1;
        else offset = groupOffset + prefixBuffer[localId - 1];
    }

    return offset;
}


__device__ inline float computeUpdatedVoxelDepthInfo(ITMVoxel_d &voxel,
                                                     const Vector4f &pt_model,
                                                     const Eigen::Matrix4f &M_d,
                                                     const Vector4f &projParams_d,
                                                     float mu,
                                                     int maxW,
                                                     const PtrStepSz<float> depth) {
    Vector4f pt_camera;
    Vector2f pt_image;
    float depth_measure, eta, oldF, newF;
    int oldW, newW;

    pt_camera = M_d * pt_model;
    if (pt_camera(2, 0) <= 0) return -1;

    pt_image.x() = projParams_d(0, 0) * pt_camera(0, 0) / pt_camera(2, 0) + projParams_d(2, 0);
    pt_image.y() = projParams_d(1, 0) * pt_camera(1, 0) / pt_camera(2, 0) + projParams_d(3, 0);

    if ((pt_image.x() < 1) || (pt_image.x() > depth.cols - 2) || (pt_image.y() < 1) || (pt_image.y() > depth.rows - 2)) return -1;

    depth_measure = depth.ptr((int) (pt_image.y() + 0.5f))[(int) (pt_image.x() + 0.5f)];
    if (depth_measure <= 0.0) return -1;

    const Vector3f xylambda((pt_image.x() - projParams_d(2, 0)) / projParams_d(0, 0),
                            (pt_image.y() - projParams_d(3, 0)) / projParams_d(1, 0),
                            1.f);
    const float lambda = xylambda.norm();
	Vector3f camera_pos(pt_camera(0, 0),pt_camera(1, 0),pt_camera(2, 0));
    eta = (-1.f) * ((1.f / lambda) * camera_pos.norm() - depth_measure);
    if (eta < -mu) return eta;

    oldF = ITMVoxel_d::valueToFloat(voxel.tsdf);
    oldW = voxel.w_depth;
    newF = MIN(1.0f, eta / mu);
    newW = 1;

    newF = oldW * oldF + newW * newF;
    newW = oldW + newW;
    newF /= newW;
    newW = MIN(newW, maxW);
	if(newW >40) {
		newW =40;
	}

    voxel.tsdf = max(-SHORTMAX,min(SHORTMAX,ITMVoxel_d::floatToValue(newF)));
    voxel.w_depth = static_cast<short>(newW);
    return eta;
}



__device__ inline void computeUpdatedVoxelDepthAndRgbInfo(ITMVoxel_d &voxel,
                                                     const Vector4f &pt_model,
                                                     const Eigen::Matrix4f &M_d,
                                                     const Vector4f &projParams_d,
                                                     const Eigen::Matrix4f &M_rgb,
                                                     const Vector4f &projParams_rgb,
                                                     float truncation_distance,
                                                     int maxW,
                                                     const PtrStepSz<float> depth,
                                                     const PtrStepSz<uchar3> rgb) {
    Vector4f pt_camera;
    Vector2f pt_image;
    float depth_measure, oldF, newF;
    int oldW, newW;

    pt_camera = M_d * pt_model;
    if (pt_camera(2, 0) <= 0) return ;

	const Vec2ida uv(__float2int_rn(projParams_d(0, 0) * pt_camera(0, 0) / pt_camera(2, 0) + projParams_d(2, 0)),
	                   __float2int_rn(projParams_d(1, 0) * pt_camera(1, 0) / pt_camera(2, 0) + projParams_d(3, 0)));
	if (uv.x() < 0 || uv.x() >= depth.cols || uv.y() < 0 || uv.y() >= depth.rows) return;
	depth_measure = depth.ptr(uv.y())[uv.x()];
    if (depth_measure <= 0.0) return ;

	const Vector3f xylambda((uv.x() - projParams_d(2, 0)) / projParams_d(0, 0),
						(uv.y() - projParams_d(3, 0)) / projParams_d(1, 0),
						1.f);
    const float lambda = xylambda.norm();
	Vector3f camera_pos(pt_camera(0, 0),pt_camera(1, 0),pt_camera(2, 0));
    // eta = (-1.f) * ((1.f / lambda) * pt_camera.norm() - depth_measure); //这里的pt_camera.norm()有很大问题啊，他的norm比正常的大1.0原来是这个导致的！
	const float sdf = (-1.f) * ((1.f / lambda) * camera_pos.norm() - depth_measure);
    if (sdf >= -truncation_distance) {
    	const float new_tsdf = fmin(1.f, sdf / truncation_distance);
    	const float current_tsdf = static_cast<float>(voxel.tsdf) * DIVSHORTMAX;
    	const int current_weight = voxel.w_depth;
    	const int add_weight = 1;

    	const float updated_tsdf = (current_weight * current_tsdf + add_weight * new_tsdf) /
								   (current_weight + add_weight);

    	int new_weight = min(current_weight + add_weight, MAX_WEIGHT);
    	if (new_weight>40){
    		new_weight=40;
    	}
    	const int new_value = max(-SHORTMAX, min(SHORTMAX, static_cast<int>(updated_tsdf * SHORTMAX)));

    	voxel.tsdf = static_cast<short>(new_value);
    	voxel.w_depth = static_cast<short>(new_weight);


    	if(sdf >= -truncation_distance&&sdf<=truncation_distance) {
    		pt_camera = M_rgb * pt_model;
    		pt_image.x() = projParams_rgb(0, 0) * pt_camera(0, 0) / pt_camera(2, 0) + projParams_rgb(2, 0);
    		pt_image.y() = projParams_rgb(1, 0) * pt_camera(1, 0) / pt_camera(2, 0) + projParams_rgb(3, 0);

    		if ((pt_image.x() < 0) || (pt_image.x() >= rgb.cols) || (pt_image.y() < 0) || (pt_image.y() >= rgb.rows)) return ;

    		uchar3& model_color = voxel.clr;
    		const uchar3 image_color = rgb.ptr((int) (pt_image.y() + 0.5f))[(int) (pt_image.x() + 0.5f)];
    		model_color.z = static_cast<uchar>(
					(oldW * model_color.z + 1 * image_color.x) /
					(oldW + 1));
    		model_color.y = static_cast<uchar>(
					(oldW * model_color.y + 1 * image_color.y) /
					(oldW + 1));
    		model_color.x = static_cast<uchar>(
					(oldW * model_color.x + 1 * image_color.z) /
					(oldW + 1));
    	}


    	// float3 res;
    	// newC = rgb.ptr((int)(pt_image.y()+0.5f))[(int)(pt_image.x()+0.5f)];
    	// oldC = voxel.clr;
    	// if(voxel.w_depth==0){
    	//     voxel.tsdf = TVoxel::floatToValue(newF);
    	//     voxel.w_depth = static_cast<short>(newW);
    	// 	voxel.clr =  newC;
    	//     return eta;
    	// }
    	// else{
    	// 	res.x = 0.2 * static_cast<float>(newC.x) + 0.8 * static_cast<float>(oldC.x);
    	// 	res.y = 0.2 * static_cast<float>(newC.y) + 0.8 * static_cast<float>(oldC.y);
    	// 	res.z = 0.2 * static_cast<float>(newC.z) + 0.8 * static_cast<float>(oldC.z);
    	// }
    	// voxel.clr.x = static_cast<uchar>(res.x);
    	// voxel.clr.y = static_cast<uchar>(res.y);
    	// voxel.clr.z = static_cast<uchar>(res.z);

    }
}


__device__ inline void computeUpdatedVoxelColorInfo(ITMVoxel_d &voxel,
                                                    const THREADPTR(Vector4f) &pt_model,
                                                    const Eigen::Matrix4f &M_rgb,
                                                    const CONSTPTR(Vector4f) &projParams_rgb,//知道rgb的外参后需要修改
                                                    float mu,
                                                    uchar maxW,
                                                    float eta,
                                                    const PtrStepSz<float> depth,
                                                    const PtrStepSz<uchar3> rgb) {
	Vector4f pt_camera; Vector2f pt_image;

	pt_camera = M_rgb * pt_model;

	pt_image.x() = projParams_rgb(0,0) * pt_camera(0,0) / pt_camera(2,0) + projParams_rgb(2,0);
	pt_image.y() = projParams_rgb(1,0) * pt_camera(1,0) / pt_camera(2,0) + projParams_rgb(3,0);

	if ((pt_image.x() < 1) || (pt_image.x() > rgb.cols - 2) || (pt_image.y() < 1) || (pt_image.y() > rgb.rows - 2)) return;

	const int current_weight = voxel.w_depth;
	int cur_weight_temp = current_weight;
	if(cur_weight_temp<40) {
		cur_weight_temp = cur_weight_temp - 1;
	}
	const int add_weight = 1;
	uchar3& model_color = voxel.clr;
	const uchar3 image_color = rgb.ptr((int)(pt_image.y()+0.5f))[(int)(pt_image.x()+0.5f)];

	model_color.z = static_cast<uchar>(
			(cur_weight_temp * model_color.z + add_weight * image_color.x) /
			(cur_weight_temp + add_weight));
	model_color.y = static_cast<uchar>(
			(cur_weight_temp * model_color.y + add_weight * image_color.y) /
			(cur_weight_temp + add_weight));
	model_color.x = static_cast<uchar>(
			(cur_weight_temp * model_color.x + add_weight * image_color.z) /
			(cur_weight_temp + add_weight));


	// Vector4f pt_camera; Vector2f pt_image;
    // uchar3 oldC,newC;
	// int newW, oldW;
	// float3 res;
	// pt_camera = M_rgb * pt_model;
	// pt_image.x() = projParams_rgb(0,0) * pt_camera(0,0) / pt_camera(2,0) + projParams_rgb(2,0);
	// pt_image.y() = projParams_rgb(1,0) * pt_camera(1,0) / pt_camera(2,0) + projParams_rgb(3,0);
	// if ((pt_image.x() < 1) || (pt_image.x() > rgb.cols - 2) || (pt_image.y() < 1) || (pt_image.y() > rgb.rows - 2)) return;

	// newC = rgb.ptr((int)(pt_image.y()+0.5f))[(int)(pt_image.x()+0.5f)];
	// oldC = voxel.clr;
	// if(voxel.w_depth==1){
	// 	voxel.clr =  newC;
	// }
	// else{
	// 	res.x = 0.2 * static_cast<float>(newC.x) + 0.8 * static_cast<float>(oldC.x);
	// 	res.y = 0.2 * static_cast<float>(newC.y) + 0.8 * static_cast<float>(oldC.y);
	// 	res.z = 0.2 * static_cast<float>(newC.z) + 0.8 * static_cast<float>(oldC.z);
	// }
	// voxel.clr.x = static_cast<uchar>(res.x);
	// voxel.clr.y = static_cast<uchar>(res.y);
	// voxel.clr.z = static_cast<uchar>(res.z);

}


struct ComputeUpdatedVoxelInfo;

struct ComputeUpdatedVoxelInfo{
    __device__ static void compute(ITMVoxel_d &voxel,
                                   const THREADPTR(Vector4f) &pt_model,
                                   const Eigen::Matrix4f &M_d,
                                   const THREADPTR(Vector4f) &projParams_d,
                                   const Eigen::Matrix4f &M_rgb,
                                   const THREADPTR(Vector4f) &projParams_rgb,
                                   float mu,
                                   int maxW,
                                   const PtrStepSz<float> depth,
                                   const PtrStepSz<uchar3> rgb) {
         // float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth);
         // if (eta >= -mu && eta <= mu)
         // 	computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, depth,rgb);
        //新的规则
        computeUpdatedVoxelDepthAndRgbInfo(voxel, pt_model, M_d, projParams_d,M_rgb, projParams_rgb, mu, maxW, depth,rgb);

    }
};


__device__ inline void buildHashAllocAndVisibleTypePP(uchar *entriesAllocType,
                                                      uchar *entriesVisibleType,
                                                      int x, int y,
                                                      Vector4s *blockCoords,
                                                      const PtrStepSz<float> depth,
                                                      Eigen::Matrix4f invM_d,
                                                      Vector4f projParams_d,
                                                      float mu,
                                                      float oneOverVoxelSize,
                                                      const CONSTPTR(ITMHashEntry) *hashTable,
                                                      float viewFrustum_min,
                                                      float viewFrustum_max,
                                                      uchar *emptyBlockEntries) {
    float depth_measure;
    unsigned int hashIdx;
    int noSteps;
    Vector4f pt_camera_f;
    Vector3f point_e, point, direction;
    Vector3s blockPos;

    depth_measure = depth.ptr(y)[x];
    if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min || (depth_measure + mu) > viewFrustum_max) return;

    pt_camera_f(2, 0) = depth_measure;//根据相机内参得到在相机坐标系的坐标
    pt_camera_f(0, 0) = pt_camera_f(2, 0) * ((float(x) - projParams_d(2, 0)) * projParams_d(0, 0));
    pt_camera_f(1, 0) = pt_camera_f(2, 0) * ((float(y) - projParams_d(3, 0)) * projParams_d(1, 0));

    float norm = sqrt(pt_camera_f(0, 0) * pt_camera_f(0, 0) + pt_camera_f(1, 0) * pt_camera_f(1, 0) + pt_camera_f(2, 0) * pt_camera_f(2, 0));//距离

    Vector4f pt_buff;
    pt_buff = pt_camera_f * (1.0f - mu / norm);
	pt_buff(0,0) = 0.0f;
	pt_buff(1,0) = 0.0f;
	pt_buff(2,0) = 0.0f;
    pt_buff(3, 0) = 1.0f;

    Vector4f temp_point = invM_d * pt_buff; //转到世界坐标系下
    point.x() = temp_point(0, 0) * oneOverVoxelSize;
    point.y() = temp_point(1, 0) * oneOverVoxelSize;
    point.z() = temp_point(2, 0) * oneOverVoxelSize;

    pt_buff = pt_camera_f * (1.0f + 10*mu / norm);
    pt_buff(3, 0) = 1.0f;
    temp_point = invM_d * pt_buff;
    point_e.x() = temp_point(0, 0) * oneOverVoxelSize;
    point_e.y() = temp_point(1, 0) * oneOverVoxelSize;
    point_e.z() = temp_point(2, 0) * oneOverVoxelSize;

    direction = point_e - point;
    norm = sqrt(direction.x() * direction.x() + direction.y() * direction.y() + direction.z() * direction.z());
    noSteps = (int) ceil(2.0f * norm);
	// noSteps = (int) ceil(norm);
    direction /= (float) (noSteps - 1);

    //add neighbouring blocks
    for (int i = 0; i < noSteps; i++) {
        blockPos.x() = floor(point.x());
        blockPos.y() = floor(point.y());
        blockPos.z() = floor(point.z());
        // Compute index in hash table.
        hashIdx = hashIndex(blockPos);

        //check if hash table contains entry
        bool isFound = false;

        ITMHashEntry hashEntry = hashTable[hashIdx];

        if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1)//<-1表示block未被分配或者已经被删除。此时，不需要更新entriesVisibleType的值
        {
            // entry has been streamed out but is visible or in memory and visible
            entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;
            emptyBlockEntries[hashIdx] = 1;
            isFound = true;
        }
        if (!isFound) {
            bool isExcess = false;
            if (hashEntry.ptr >= -1) {
                while (hashEntry.offset >= 1) {
                    hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
                    hashEntry = hashTable[hashIdx];

                    if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {
                        //entry has been streamed out but is visible or in memory and visible
                        entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;
                        emptyBlockEntries[hashIdx] = 1;
                        isFound = true;
                        break;
                    }
                }
                isExcess = true;
            }

            if (!isFound) {
                // Needs allocation.
                entriesAllocType[hashIdx] = isExcess ? 2 : 1;

                if (!isExcess) {
                    entriesVisibleType[hashIdx] = 1;
                    emptyBlockEntries[hashIdx] = 1;
                }
                blockCoords[hashIdx] = Vector4s(blockPos.x(), blockPos.y(), blockPos.z(), 1);
            }
        }
        point += direction;
    }
}

template<bool useSwapping>
__device__ inline void checkPointVisibility(THREADPTR(bool) &isVisible,
                                            THREADPTR(bool) &isVisibleEnlarged,
                                            const THREADPTR(Vector4f) &pt_image,
                                            const Eigen::Matrix4f &M_d,
                                            const CONSTPTR(Vector4f) &projParams_d,
                                            const CONSTPTR(Vector2i) &imgSize) {
    Vector4f pt_buff;

    pt_buff = M_d * pt_image;

    if (pt_buff(2, 0) < 1e-10f) return;

    pt_buff(0, 0) = projParams_d(0, 0) * pt_buff(0, 0) / pt_buff(2, 0) + projParams_d(2, 0);
    pt_buff(1, 0) = projParams_d(1, 0) * pt_buff(1, 0) / pt_buff(2, 0) + projParams_d(3, 0);

    if (pt_buff(0, 0) >= 0 && pt_buff(0, 0) < imgSize.x() && pt_buff(1, 0) >= 0 && pt_buff(1, 0) < imgSize.y()) {
        isVisible = true;
        isVisibleEnlarged = true;
    } else if (useSwapping) {
        Vector4i lims;
        lims(0, 0) = -imgSize.x() / 8;
        lims(1, 0) = imgSize.x() + imgSize.x() / 8;
        lims(2, 0) = -imgSize.y() / 8;
        lims(3, 0) = imgSize.y() + imgSize.y() / 8;

        if (pt_buff(0, 0) >= lims(0, 0)
            && pt_buff(0, 0) < lims(1, 0)
            && pt_buff(1, 0) >= lims(2, 0)
            && pt_buff(1, 0) < lims(3, 0))
            isVisibleEnlarged = true;
    }
}

template<bool useSwapping>
__device__ inline void checkBlockVisibility(THREADPTR(bool) &isVisible,
                                            THREADPTR(bool) &isVisibleEnlarged,
                                            const int3 hashPos,
                                            const Eigen::Matrix4f &M_d,
                                            const CONSTPTR(Vector4f) &projParams_d,
                                            const CONSTPTR(float) &voxelSize,
                                            const CONSTPTR(Vector2i) &imgSize) {
    Vector4f pt_image;
    float factor = (float) SDF_BLOCK_SIZE * voxelSize;

    isVisible = false;
    //isVisibleEnlarged = false;

    // 0 0 0
    pt_image(0, 0) = (float) hashPos.x * factor;
    pt_image(1, 0) = (float) hashPos.y * factor;
    pt_image(2, 0) = (float) hashPos.z * factor;
    pt_image(3, 0) = 1.0f;
    checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
    if (isVisible) return;

    // 0 0 1
    pt_image(2, 0) += factor;
    checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
    if (isVisible) return;

    // 0 1 1
    pt_image(1, 0) += factor;
    checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
    if (isVisible) return;

    // 1 1 1
    pt_image(0, 0) += factor;
    checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
    if (isVisible) return;

    // 1 1 0
    pt_image(2, 0) -= factor;
    checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
    if (isVisible) return;

    // 1 0 0
    pt_image(1, 0) -= factor;
    checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
    if (isVisible) return;

    // 0 1 0
    pt_image(0, 0) -= factor;
    pt_image(1, 0) += factor;
    checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
    if (isVisible) return;

    // 1 0 1
    pt_image(0, 0) += factor;
    pt_image(1, 0) -= factor;
    pt_image(2, 0) += factor;
    checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
    if (isVisible) return;
}

template<bool useSwapping>
__device__ inline void checkCsmBlockVisibility(THREADPTR(bool) &isVisible, THREADPTR(bool) &isVisibleEnlarged,
	const int3 hashPos, const Eigen::Matrix4f & M_d, const CONSTPTR(Vector4f) &projParams_d,
	const CONSTPTR(float) &voxelSize, const CONSTPTR(Vector2i) &imgSize,int* csm_size)
{
	Vector4f pt_iage;
	float factor =(float)SDF_BLOCK_SIZE * voxelSize;//一个block的边长

	isVisible = false; isVisibleEnlarged = false;
	float factor_x =(csm_size[3]-csm_size[0]) /8;
	float factor_y =(csm_size[4]-csm_size[1]) /8;
	float factor_z =(csm_size[5]-csm_size[2]) /8;

	//这个三维点是否在当前子图下或在相机的扩大可见范围中
	if(hashPos.x>csm_size[0]-factor_x && hashPos.x<csm_size[3]+factor_x && hashPos.y>csm_size[1]-factor_y && hashPos.y<csm_size[4]+factor_y &&
			hashPos.z>csm_size[2]-factor_z && hashPos.z<csm_size[5]+factor_z){
		isVisible = true;
		isVisibleEnlarged=true;
	}
}