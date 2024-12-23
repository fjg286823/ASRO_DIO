// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "utilitys.h"
#include "ITMVoxelTypes.h"
#include "ITMVoxelBlockHash.h"
#include "ITMRenderState_VH.h"
#include "ITMScene.h"

namespace DWIO {


    class ITMSceneReconstructionEngine_CUDA {
    private:
        void *allocationTempData_device;
        void *allocationTempData_host;
        unsigned char *entriesAllocType_device;
        Vector4s *blockCoords_device;

    public:
        void ResetScene(ITMScene *scene);

        void AllocateScene(ITMScene* scene, const cv::cuda::GpuMat &depth_map, const Eigen::Matrix4d &pose,
                           ITMRenderState_VH *renderState_vh, Vector4f camera_intrinsic, float truncation_distance, int *csm_size,
                           bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

        void AllocateSceneFromDepth(ITMScene *scene, const cv::cuda::GpuMat &depth_map, const Eigen::Matrix4d &pose,
                                    ITMRenderState_VH *renderState_vh, Vector4f camera_intrinsic,float truncation_distance,int* csm_size);

        void IntegrateIntoScene(ITMScene *scene, const cv::cuda::GpuMat &depth_map,
                                const cv::cuda::GpuMat &rgb, const Eigen::Matrix4d &pose_inv,
                                ITMRenderState_VH *renderState_vh, Vector4f depth_intrinsic,  Vector4f color_intrinsic,const Eigen::Matrix4d& extrinic_from_d_2_rgb,float truncation_distance);

        void computeMapBlock(ITMScene *scene, Vector3s *blockPos_device);

        void SwapAllBlocks(ITMScene *scene, ITMRenderState_VH *renderState_vh, int *NeedToSwapIn);

        void showHashTableAndVoxelAllocCondition(ITMScene *scene,ITMRenderState_VH *renderState_vh);
        
        ITMSceneReconstructionEngine_CUDA(void);

        ~ITMSceneReconstructionEngine_CUDA(void);
    };


}
