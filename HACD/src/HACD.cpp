#include "HACD.h"
#include <stdlib.h>
#include <string.h>
#include "PlatformConfigHACD.h"

#include "dgMeshEffect.h"
#include "dgConvexHull3d.h"
#include "MergeHulls.h"

#pragma warning(disable:4100 4996)

using namespace hacd;

namespace HACD
{

class MyHACD_API : public HACD_API, public UANS::UserAllocated
{
public:
	MyHACD_API(void)
	{
	}
	virtual ~MyHACD_API(void)
	{
		releaseHACD();
	}

	virtual hacd::HaU32	performHACD(const Desc &desc) 
	{
		hacd::HaU32 ret = 0;
		releaseHACD();
		if ( desc.mVertexCount )
		{
			{
				dgMeshEffect mesh(true);

				float normal[3] = { 0,1,0 };
				float uv[2] = { 0,0 };

				hacd::HaI32 *faceIndexCount = (hacd::HaI32 *)HACD_ALLOC(sizeof(hacd::HaI32)*desc.mTriangleCount);
				hacd::HaI32 *dummyIndex = (hacd::HaI32 *)HACD_ALLOC(sizeof(hacd::HaI32)*desc.mTriangleCount*3);

				for (hacd::HaU32 i=0; i<desc.mTriangleCount; i++)
				{
					faceIndexCount[i] = 3;
					dummyIndex[i*3+0] = 0;
					dummyIndex[i*3+1] = 0;
					dummyIndex[i*3+2] = 0;
				}

				mesh.BuildFromVertexListIndexList(desc.mTriangleCount,faceIndexCount,dummyIndex,
					desc.mVertices,sizeof(hacd::HaF32)*3,(const hacd::HaI32 *const)desc.mIndices,
					normal,sizeof(hacd::HaF32)*3,dummyIndex,
					uv,sizeof(hacd::HaF32)*2,dummyIndex,
					uv,sizeof(hacd::HaF32)*2,dummyIndex);

				dgMeshEffect *result = mesh.CreateConvexApproximation(0.1f,desc.mMaxHullCount);

				if ( result )
				{
					// now we build hulls for each connected surface...
					dgPolyhedra segment;
					result->BeginConectedSurface();
					if ( result->GetConectedSurface(segment))
					{
						dgMeshEffect *solid = HACD_NEW(dgMeshEffect)(segment,*result);
						while ( solid )
						{
							dgConvexHull3d *hull = solid->CreateConvexHull(0.00001,desc.mMaxHullVertices);
							if ( hull )
							{
								Hull h;
								h.mVertexCount = hull->GetVertexCount();
								h.mVertices = (hacd::HaF32 *)HACD_ALLOC( sizeof(hacd::HaF32)*3*h.mVertexCount);
								for (hacd::HaU32 i=0; i<h.mVertexCount; i++)
								{
									hacd::HaF32 *dest = (hacd::HaF32 *)&h.mVertices[i*3];
									const dgBigVector &source = hull->GetVertex(i);
									dest[0] = (hacd::HaF32)source.m_x;
									dest[1] = (hacd::HaF32)source.m_y;
									dest[2] = (hacd::HaF32)source.m_z;
								}

								h.mTriangleCount = hull->GetCount();
								hacd::HaU32 *destIndices = (hacd::HaU32 *)HACD_ALLOC(sizeof(hacd::HaU32)*3*h.mTriangleCount);
								h.mIndices = destIndices;
			
								dgList<dgConvexHull3DFace>::Iterator iter(*hull);
								for (iter.Begin(); iter; iter++)
								{
									dgConvexHull3DFace &face = (*iter);
									destIndices[0] = face.m_index[0];
									destIndices[1] = face.m_index[1];
									destIndices[2] = face.m_index[2];
									destIndices+=3;
								}

								mHulls.push_back(h);

								// save it!
								delete hull;
							}

							delete solid;
							solid = NULL;
							dgPolyhedra nextSegment;
							hacd::HaI32 moreSegments = result->GetConectedSurface(nextSegment);
							if ( moreSegments )
							{
								solid = HACD_NEW(dgMeshEffect)(nextSegment,*result);
							}
							else
							{
								result->EndConectedSurface();
							}
						}
					}

					delete result;
				}
				ret= (HaU32)mHulls.size();
			}
		}

		if ( ret && ret > desc.mMaxHullCount )
		{
			MergeHullsInterface *mhi = createMergeHullsInterface();
			if ( mhi )
			{
				MergeHullVector inputHulls;
				MergeHullVector outputHulls;
				for (hacd::HaU32 i=0; i<ret; i++)
				{
					Hull &h = mHulls[i];
					MergeHull mh;
					mh.mTriangleCount = h.mTriangleCount;
					mh.mVertexCount = h.mVertexCount;
					mh.mVertices = h.mVertices;
					mh.mIndices = h.mIndices;
					inputHulls.push_back(mh);
				}

				mhi->mergeHulls(inputHulls,outputHulls,desc.mMaxHullCount);

				for (HaU32 i=0; i<ret; i++)
				{
					Hull &h = mHulls[i];
					releaseHull(h);
				}
				mHulls.clear();

				for (hacd::HaU32 i=0; i<outputHulls.size(); i++)
				{
					Hull h;
					const MergeHull &mh = outputHulls[i];
					h.mTriangleCount =  mh.mTriangleCount;
					h.mVertexCount = mh.mVertexCount;
					h.mIndices = (HaU32 *)HACD_ALLOC(sizeof(HaU32)*3*h.mTriangleCount);
					h.mVertices = (HaF32 *)HACD_ALLOC(sizeof(HaF32)*3*h.mVertexCount);
					memcpy((HaU32 *)h.mIndices,mh.mIndices,sizeof(HaU32)*3*h.mTriangleCount);
					memcpy((HaF32 *)h.mVertices,mh.mVertices,sizeof(HaF32)*3*h.mVertexCount);
					mHulls.push_back(h);
				}

				ret = (HaU32)mHulls.size();

				mhi->release();
			}
		}

		return ret;
	}

	void releaseHull(Hull &h)
	{
		HACD_FREE((void *)h.mIndices);
		HACD_FREE((void *)h.mVertices);
		h.mIndices = NULL;
		h.mVertices = NULL;
	}

	virtual const Hull		*getHull(hacd::HaU32 index)  const
	{
		const Hull *ret = NULL;
		if ( index < mHulls.size() )
		{
			ret = &mHulls[index];
		}
		return ret;
	}

	virtual void	releaseHACD(void) // release memory associated with the last HACD request
	{
		for (hacd::HaU32 i=0; i<mHulls.size(); i++)
		{
			releaseHull(mHulls[i]);
		}
		mHulls.clear();
	}


	virtual void release(void) // release the HACD_API interface
	{
		delete this;
	}

	virtual hacd::HaU32	getHullCount(void)
	{
		return (hacd::HaU32) mHulls.size(); 
	}

private:
	STDNAME::vector< Hull >	mHulls;
};

HACD_API * createHACD_API(void)
{
	MyHACD_API *m = HACD_NEW(MyHACD_API);
	return static_cast<HACD_API *>(m);
}


};



