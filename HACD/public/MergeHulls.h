#ifndef MERGE_HULLS_H

#define MERGE_HULLS_H

#include "PlatformConfigHACD.h"

namespace HACD
{

class MergeHull
{
public:
	hacd::HaU32			mTriangleCount;
	hacd::HaU32			mVertexCount;
	const hacd::HaF32	*mVertices;
	const hacd::HaU32	*mIndices;
};

typedef STDNAME::vector< MergeHull > MergeHullVector;

class MergeHullsInterface
{
public:
	// Merge these input hulls.
	virtual hacd::HaU32 mergeHulls(const MergeHullVector &inputHulls,
									MergeHullVector &outputHulls,
									hacd::HaU32	mergeHullCount) = 0;


	virtual void release(void) = 0;

protected:
	virtual ~MergeHullsInterface(void)
	{

	}

};

MergeHullsInterface * createMergeHullsInterface(void);

}; // end of HACD namespace

#endif
