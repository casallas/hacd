#ifndef HACD_H

#define HACD_H

#include "PlatformConfigHACD.h"
#include <stdlib.h>

namespace HACD
{


class HACD_API
{
public:
	class Desc
	{
	public:
		Desc(void)
		{
			init();
		}

		hacd::HaU32			mTriangleCount;
		hacd::HaU32			mVertexCount;
		const hacd::HaF32	*mVertices;
		const hacd::HaU32	*mIndices;
		hacd::HaU32			mMaxHullCount;
		hacd::HaU32			mMaxMergeHullCount;
		hacd::HaU32			mMaxHullVertices;
		hacd::HaF32			mConcavity;
		void init(void)
		{
			mTriangleCount = 0;
			mVertexCount = 0;
			mVertices = NULL;
			mIndices = NULL;
			mMaxHullCount = 256;
			mMaxMergeHullCount = 256;
			mMaxHullVertices = 64;
			mConcavity = 0.2f;
		}
	};

	class Hull
	{
	public:
		hacd::HaU32			mTriangleCount;
		hacd::HaU32			mVertexCount;
		const hacd::HaF32	*mVertices;
		const hacd::HaU32	*mIndices;
	};

	virtual hacd::HaU32		performHACD(const Desc &desc) = 0;
	virtual hacd::HaU32		getHullCount(void) = 0;
	virtual const Hull		*getHull(hacd::HaU32 index) const = 0;
	virtual void			releaseHACD(void) = 0; // release memory associated with the last HACD request
	

	virtual void			release(void) = 0; // release the HACD_API interface
protected:
	virtual ~HACD_API(void)
	{

	}
};

HACD_API * createHACD_API(void);

};

#endif
