
#pragma once

#define THREADPTR(x) x
#define CONSTPTR(x) x

#include "ITMVoxelTypes.h"
#include "ITMVoxelBlockHash.h"
#include "utilitys.h"
#include "ITMVoxelBlockHash.h"

template<typename T>
__device__ inline int hashIndex(const THREADPTR(T) &blockPos) {
    return (((uint) blockPos.x() * 73856093u) ^ ((uint) blockPos.y() * 19349669u) ^ ((uint) blockPos.z() * 83492791u)) & (uint) SDF_HASH_MASK;
}

__device__ inline int pointToVoxelBlockPos(const THREADPTR(Vector3i) &point, Vector3s &blockPos) {
    blockPos.x() = ((point.x() < 0) ? point.x() - SDF_BLOCK_SIZE + 1 : point.x()) / SDF_BLOCK_SIZE;
    blockPos.y() = ((point.y() < 0) ? point.y() - SDF_BLOCK_SIZE + 1 : point.y()) / SDF_BLOCK_SIZE;
    blockPos.z() = ((point.z() < 0) ? point.z() - SDF_BLOCK_SIZE + 1 : point.z()) / SDF_BLOCK_SIZE;

    return point.x() + (point.y() - blockPos.x()) * SDF_BLOCK_SIZE + (point.z() - blockPos.y()) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE -
           blockPos.z() * SDF_BLOCK_SIZE3;
}


__device__ inline ITMVoxel_d readVoxel(const CONSTPTR(ITMVoxel_d) *voxelData,
                                   const CONSTPTR(DWIO::ITMVoxelBlockHash::IndexData) *voxelIndex,
                                   const THREADPTR(Vector3i) &point,
                                   THREADPTR(int) &vmIndex,
                                   THREADPTR(DWIO::ITMVoxelBlockHash::IndexCache) &cache) {
    Vector3s blockPos;
    int linearIdx = pointToVoxelBlockPos(point, blockPos);

    if IS_EQUAL3(cache.blockPos, blockPos) {
        vmIndex = true;
        return voxelData[cache.blockPtr + linearIdx];
    }

    int hashIdx = hashIndex(blockPos);

    while (true) {
        ITMHashEntry hashEntry = voxelIndex[hashIdx];

        if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
            cache.blockPos.x = blockPos.x();
            cache.blockPos.y = blockPos.y();
            cache.blockPos.z = blockPos.z();
            cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
            vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound
        
            return voxelData[cache.blockPtr + linearIdx];
        }

        if (hashEntry.offset < 1) break;
        hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
    }

    vmIndex = false;
    return ITMVoxel_d();
}



__device__ inline ITMVoxel_d readVoxelCpu(const CONSTPTR(ITMVoxel_d) *voxelData,
                                   const CONSTPTR(DWIO::ITMVoxelBlockHash::IndexData) *voxelIndex,
                                   const THREADPTR(Vector3i) &point,
                                   THREADPTR(int) &vmIndex,
                                   THREADPTR(DWIO::ITMVoxelBlockHash::IndexCache) &cache) {
    Vector3s blockPos;
    int linearIdx = pointToVoxelBlockPos(point, blockPos);

    int hashIdx = hashIndex(blockPos);
    if IS_EQUAL3(cache.blockPos, blockPos) {
        vmIndex = true;
        return voxelData[hashIdx*SDF_BLOCK_SIZE3 + linearIdx];
    }

    while (true) {
        ITMHashEntry hashEntry = voxelIndex[hashIdx];

        if (IS_EQUAL3(hashEntry.pos, blockPos) ) {
            cache.blockPos.x = blockPos.x();
            cache.blockPos.y = blockPos.y();
            cache.blockPos.z = blockPos.z();
            cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
            vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound

            return voxelData[hashIdx*SDF_BLOCK_SIZE3 + linearIdx];
        }

        if (hashEntry.offset < 1) break;
        hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
    }

    vmIndex = false;
    return ITMVoxel_d();
}


__device__ inline ITMVoxel_d readVoxelCpu(const CONSTPTR(ITMVoxel_d) *voxelData,
                                   const CONSTPTR(DWIO::ITMVoxelBlockHash::IndexData) *voxelIndex,
                                   Vector3i point,
                                   THREADPTR(int) &vmIndex) {
    DWIO::ITMVoxelBlockHash::IndexCache cache;
    return readVoxelCpu(voxelData, voxelIndex, point, vmIndex, cache);
}


__device__ inline ITMVoxel_d readVoxel(const CONSTPTR(ITMVoxel_d) *voxelData,
                                   const CONSTPTR(DWIO::ITMVoxelBlockHash::IndexData) *voxelIndex,
                                   Vector3i point,
                                   THREADPTR(int) &vmIndex) {
    DWIO::ITMVoxelBlockHash::IndexCache cache;
    return readVoxel(voxelData, voxelIndex, point, vmIndex, cache);
}


__device__ inline ITMVoxel_d readVoxel(const CONSTPTR(ITMVoxel_d) *voxelData,
                                   const CONSTPTR(DWIO::ITMVoxelBlockHash::IndexData) *voxelIndex,
                                   Vector3i point,
                                   THREADPTR(bool) &foundPoint) {
    int vmIndex;
    DWIO::ITMVoxelBlockHash::IndexCache cache;
    ITMVoxel_d result = readVoxel(voxelData, voxelIndex, point, vmIndex, cache);
    foundPoint = vmIndex != 0;
    return result;
}


__device__ inline int16_t readFromSDF_float_uninterpolated(const ITMVoxel_d *voxelData,
                                                           const ITMHashEntry *voxelIndex,
                                                           Vector3f point,
                                                           THREADPTR(int) &vmIndex) {
    ITMVoxel_d res = readVoxel(voxelData, voxelIndex, Vector3i((int) ROUND(point.x()), (int) ROUND(point.y()), (int) ROUND(point.z())), vmIndex);
    return res.tsdf;
}