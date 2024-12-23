// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMRenderState_VH.h"
#include "ITMScene.h"
#include "ITMVoxelBlockHash.h"
#include "ITMVoxelTypes.h"
#include "utilitys.h"

namespace DWIO {

    class ITMSwappingEngine_CUDA {
    private:
        int *noNeededEntries_device, *noAllocatedVoxelEntries_device;
        int *entriesToClean_device;
        int *moveInEntries_device;
        uchar *blockEmptyVerify;

        int LoadFromGlobalMemory(ITMScene *scene);

    public:
        void IntegrateGlobalIntoLocal(ITMScene *scene, ITMRenderState_VH *renderState, bool updateFlag);

        void SaveToGlobalMemory(ITMScene *scene, ITMRenderState_VH *renderState);

        void TransferGlobalMap(ITMScene *scene);


        void MoveVoxelToGlobalMemorey(ITMScene *scene, ITMRenderState_VH *renderState);

        ITMSwappingEngine_CUDA(void);

        ~ITMSwappingEngine_CUDA(void);
    };

}

