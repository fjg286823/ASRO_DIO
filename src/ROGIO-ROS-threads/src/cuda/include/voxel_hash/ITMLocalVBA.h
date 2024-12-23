#pragma once
#include<iostream>
#include "ITMVoxelTypes.h"
#include "MemoryDeviceType.h"
#include "CUDADefines.h"
//如果不交换这个一整个都能去掉
namespace DWIO {
    /** \brief
    Stores the actual voxel content that is referred to by a
    ITMLib::ITMHashTable. on GPU
    */
    class ITMLocalVBA {
    private:
        ITMVoxel_d *voxelBlocks;
        int *allocationList;


    public:
        inline ITMVoxel_d *GetVoxelBlocks(void) { return voxelBlocks; }

        inline const ITMVoxel_d *GetVoxelBlocks(void) const { return voxelBlocks; }

        int *GetAllocationList(void) { return allocationList; }

        int lastFreeBlockId;

        int allocatedSize;


        ITMLocalVBA(int noBlocks, int blockSize) {
            allocatedSize = noBlocks * blockSize;

            DWIOcudaSafeCall(cudaMalloc((void **)&voxelBlocks, allocatedSize * sizeof(ITMVoxel_d)));
            DWIOcudaSafeCall(cudaMalloc((void **)&allocationList, noBlocks * sizeof(int)));

            // voxelBlocks = new DWIO::MemoryBlock<ITMVoxel_d>(allocatedSize, memoryType);
            // allocationList = new DWIO::MemoryBlock<int>(noBlocks, memoryType);
            std::cout<<"use :"<<allocatedSize * sizeof(ITMVoxel_d) /1024/1024<<" MB"<<std::endl;
        }

        ~ITMLocalVBA(void) {
            cudaFree(voxelBlocks);
            cudaFree(allocationList);
        }

        ITMLocalVBA(const ITMLocalVBA &);

        ITMLocalVBA &operator=(const ITMLocalVBA &);
    };
}
