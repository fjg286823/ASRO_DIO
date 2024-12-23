
#pragma once

#include <stdlib.h>
#include <fstream>
#include <iostream>

#include <cuda.h>
#include <cuda_runtime_api.h>


#include <device_launch_parameters.h>
#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>

#include "MemoryDeviceType.h"
#include "CUDADefines.h"//里面包含一个函数用于检查cuda函数调用是否正确

#define SDF_BLOCK_SIZE 8                // SDF block size
#define SDF_BLOCK_SIZE3 512                // SDF_BLOCK_SIZE3 = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE

#define SDF_LOCAL_BLOCK_NUM 0x20000        // Number of locally stored blocks, 0x60000

// #define SDF_BUCKET_NUM 0x200000            // Number of Hash Bucket, should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM, SDF_HASH_MASK = SDF_BUCKET_NUM - 1
// #define SDF_HASH_MASK 0x1fffff            // Used for get hashing value of the bucket index,  SDF_HASH_MASK = SDF_BUCKET_NUM - 1
// #define SDF_EXCESS_LIST_SIZE 0x20000    // Size of excess list, used to handle collisions. Also max offset (unsigned short) value.

#define SDF_BUCKET_NUM 0x40000            // Number of Hash Bucket, should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM, SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_HASH_MASK 0x3ffff            // Used for get hashing value of the bucket index,  SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_EXCESS_LIST_SIZE 0x8000    // Size of excess list, used to handle collisions. Also max offset (unsigned short) value.

#define SDF_TRANSFER_BLOCK_NUM 0x2000    // Maximum number of blocks transfered in one swap operation

/** \brief
	A single entry in the hash table.
*/
struct ITMHashEntry
{
    /** Position of the corner of the 8x8x8 volume, that identifies the entry. */
    int3 pos;
    /** Offset in the excess list. */
    int offset;

    /** Pointer to the voxel block array.
        - >= 0 identifies an actual allocated entry in the voxel block array
        - -1 identifies an entry that has been removed (swapped out)
        - <-1 identifies an unallocated block
    */
    int ptr;
};

namespace DWIO
{
    /** \brief
    This is the central class for the voxel block hash
    implementation. It contains all the data needed on the CPU
    and a pointer to the data structure on the GPU.
    */
    class ITMVoxelBlockHash {
    public:
        typedef ITMHashEntry IndexData;

        struct IndexCache {
            int3 blockPos;
            int blockPtr;
            __device__ IndexCache(void) {
                blockPos.x = 0x7fffffff;
                blockPos.y = 0x7fffffff;
                blockPos.z = 0x7fffffff;
                blockPtr = -1;
            }
        };

        /** Maximum number of total entries. */
        static const int noTotalEntries = SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE;
        static const int voxelBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

    private:
        int lastFreeExcessListId;

        /** The actual data in the hash table. */
        ITMHashEntry *hashEntries;
        ITMHashEntry *hashEntries_cpu;

        /** Identifies which entries of the overflow
        list are allocated. This is used if too
        many hash collisions caused the buckets to
        overflow.
        */
        int *excessAllocationList;

        MemoryDeviceType memoryType;

    public:
        ITMVoxelBlockHash(MemoryDeviceType memoryType) {
            this->memoryType = memoryType;

            switch (memoryType) {
                case MEMORYDEVICE_CPU:
                    cudaMallocHost((void **)&hashEntries, noTotalEntries * sizeof(ITMHashEntry));
                    break;
                case MEMORYDEVICE_CUDA: {
                    DWIOcudaSafeCall(cudaMalloc((void **)&hashEntries, noTotalEntries * sizeof(ITMHashEntry)));
                    break;
                }
            }
            cudaMallocHost((void **)&hashEntries_cpu, noTotalEntries * sizeof(ITMHashEntry));
            cudaMallocHost((void **)&excessAllocationList, SDF_EXCESS_LIST_SIZE * sizeof(int));

            // hashEntries = new DWIO::MemoryBlock<ITMHashEntry>(noTotalEntries, memoryType);
            // hashEntries_cpu = new DWIO::MemoryBlock<ITMHashEntry>(noTotalEntries, MEMORYDEVICE_CPU);
            // excessAllocationList = new DWIO::MemoryBlock<int>(SDF_EXCESS_LIST_SIZE, memoryType);
        }

        ~ITMVoxelBlockHash(void) {
            switch (this->memoryType) {
                case MEMORYDEVICE_CPU:
                    cudaFreeHost(hashEntries);
                    break;
                case MEMORYDEVICE_CUDA: 
                    cudaFree(hashEntries);
                    break;
            }
            cudaFreeHost(hashEntries_cpu);
            cudaFreeHost(excessAllocationList);
        }

        /** Get the list of actual entries in the hash table. */
        const ITMHashEntry *GetEntries(void) const { return hashEntries; }

        ITMHashEntry *GetEntries(void) { return hashEntries; }

        ITMHashEntry *GetEntriesCpu(void) { 
            return hashEntries_cpu; 
        }

        void GpuToCpuHashData(void)
        {
            DWIOcudaSafeCall(cudaMemcpy(hashEntries_cpu,hashEntries,sizeof(ITMHashEntry)*noTotalEntries,cudaMemcpyDeviceToHost));
        }
        

        const IndexData *getIndexData(void) const { return hashEntries; }

        IndexData *getIndexData(void) { return hashEntries; }


        const int *GetExcessAllocationList(void) const { return excessAllocationList; }

        int *GetExcessAllocationList(void) { return excessAllocationList; }

        int GetLastFreeExcessListId(void) { return lastFreeExcessListId; }

        void SetLastFreeExcessListId(int lastFreeExcessListId) { this->lastFreeExcessListId = lastFreeExcessListId; }


        /** Maximum number of total entries. */
        int getNumAllocatedVoxelBlocks(void) { return SDF_LOCAL_BLOCK_NUM; }

        int getVoxelBlockSize(void) { return SDF_BLOCK_SIZE3; }

        // Suppress the default copy constructor and assignment operator
        ITMVoxelBlockHash(const ITMVoxelBlockHash &);

        ITMVoxelBlockHash &operator=(const ITMVoxelBlockHash &);
    };
}