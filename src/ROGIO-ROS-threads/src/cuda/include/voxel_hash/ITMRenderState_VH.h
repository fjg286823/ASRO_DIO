// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>


#include "ITMVoxelBlockHash.h"

namespace DWIO {
    /** \brief
        Stores the render state used by the SceneReconstruction
        and visualisation engines, as used by voxel hashing.
    */
    class ITMRenderState_VH {
    private:
        MemoryDeviceType memoryType;

        /** A list of "visible entries", that are currently
        being processed by the tracker.
        */
        int *visibleEntryIDs;

        /** A list of "visible entries", that are
        currently being processed by integration
        and tracker.
        */
        // 0：表示该体素块是空闲的，没有被分配，也没有被使用
        // 1：表示该体素块是活跃的，已经被分配，且在当前帧中被观察到
        // 2：应该是可见但在cpu上
        // 3：表示该体素块是删除的，已经被分配，但在之前的帧中被判定为不可见，需要被释放
        uchar *entriesVisibleType;
        uchar *emptyBlockEntries;//表明哪些是由射线投射得到的真正有值的block

    public:
        /** Number of entries in the live list. */
        int noVisibleEntries;

        ITMRenderState_VH(int noTotalEntries, MemoryDeviceType memoryType = MEMORYDEVICE_CPU) {
            this->memoryType = memoryType;

            switch (memoryType) {
                case MEMORYDEVICE_CPU:
                    cudaMallocHost((void **)&visibleEntryIDs, SDF_LOCAL_BLOCK_NUM * sizeof(int));
                    cudaMallocHost((void **)&entriesVisibleType, noTotalEntries * sizeof(uchar));
                    cudaMallocHost((void **)&emptyBlockEntries, noTotalEntries * sizeof(uchar));
                    break;
                case MEMORYDEVICE_CUDA: {
                    DWIOcudaSafeCall(cudaMalloc((void **)&visibleEntryIDs, SDF_LOCAL_BLOCK_NUM * sizeof(int)));
                    DWIOcudaSafeCall(cudaMalloc((void **)&entriesVisibleType, noTotalEntries * sizeof(uchar)));
                    DWIOcudaSafeCall(cudaMalloc((void **)&emptyBlockEntries, noTotalEntries * sizeof(uchar)));
                    break;
                }
            }
            
            // visibleEntryIDs = new DWIO::MemoryBlock<int>(SDF_LOCAL_BLOCK_NUM, memoryType);
            // entriesVisibleType = new DWIO::MemoryBlock<uchar>(noTotalEntries, memoryType);
            // emptyBlockEntries = new DWIO::MemoryBlock<uchar>(noTotalEntries, memoryType);
            noVisibleEntries = 0;
        }

        ~ITMRenderState_VH() {
            switch (memoryType) {
                case MEMORYDEVICE_CPU:
                    cudaFreeHost(visibleEntryIDs);
                    cudaFreeHost(entriesVisibleType);
                    cudaFreeHost(emptyBlockEntries);
                    break;
                case MEMORYDEVICE_CUDA: {
                    cudaFree(visibleEntryIDs);
                    cudaFree(entriesVisibleType);
                    cudaFree(emptyBlockEntries);
                    break;
                }
            }
        }

        int *GetVisibleEntryIDs(void) { return visibleEntryIDs; }

        uchar *GetEntriesVisibleType(void) { return entriesVisibleType; }

        uchar *GetEmptyBlockEntries(void) { return emptyBlockEntries; }

    };
} 
