
#pragma once

#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"

namespace DWIO
{
	/** \brief
	Represents the 3D world model as a hash of small voxel
	blocks
	*/
	class ITMScene
	{
	public:

		/** Hash table to reference the 8x8x8 blocks */
		ITMVoxelBlockHash index;//hash表

		/** Current local content of the 8x8x8 voxel blocks -- stored host or device */
		ITMLocalVBA localVBA;

		/** Global content of the 8x8x8 voxel blocks -- stored on host only */
		ITMGlobalCache* globalCache;


		int maxW = 128;
		float voxel_resolution = 20.0f;
		float viewFrustum_min, viewFrustum_max;

		ITMScene( bool _useSwapping,float voxel_size ,MemoryDeviceType _memoryType)
			: index(_memoryType), localVBA(index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize())
		{
			if (_useSwapping) globalCache = new ITMGlobalCache();
			else globalCache = NULL;
			viewFrustum_min = 600.0f;
			viewFrustum_max = 3000.0f;
			voxel_resolution = voxel_size;
		}

		~ITMScene(void)
		{
			if (globalCache != NULL) delete globalCache;
		}

		// Suppress the default copy constructor and assignment operator
		ITMScene(const ITMScene&);
		ITMScene& operator=(const ITMScene&);
	};
}
