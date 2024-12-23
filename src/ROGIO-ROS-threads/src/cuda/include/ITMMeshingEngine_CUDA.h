// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMVoxelBlockHash.h"
#include "ITMVoxelTypes.h"
#include "ITMMesh.h"
#include "ITMScene.h"

#include "utilitys.h"

namespace DWIO {

    class ITMMeshingEngine_CUDA {
    private:
        unsigned int *noTriangles_device;
        Vector4s *visibleBlockGlobalPos_device;

    public:
        void MeshScene(ITMMesh *mesh, const ITMScene *scene,int& TotalTriangles);

        ITMMeshingEngine_CUDA(void);

        ~ITMMeshingEngine_CUDA(void);
    };


}


