/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __dgPolyhedra__
#define __dgPolyhedra__

//#include "dgStdafx.h"
#include "dgList.h"
#include "dgTree.h"
#include "dgHeap.h"

class dgEdge;
class dgPlane;
class dgSphere;
class dgMatrix;
class dgPolyhedra;


typedef hacd::HaI64 dgEdgeKey;

/*
class dgPolyhedraDescriptor
{
	public:

	 dgPolyhedraDescriptor (const dgPolyhedra& polyhedra);
	 ~dgPolyhedraDescriptor ();
	 void Update (const dgPolyhedra& polyhedra);

	hacd::HaI32 m_faceCount;							 // total face count including openfaces
	hacd::HaI32 m_edgeCount;							 // total edge count count including openfaces
	hacd::HaI32 m_vertexCount;						 // total vertex count including openfaces
	hacd::HaI32 m_maxVertexIndex;
	dgList<dgEdge*> m_unboundedLoops;
};
*/

class dgEdge
{
	public:
	dgEdge ();
	dgEdge (hacd::HaI32 vertex, hacd::HaI32 face, hacd::HaU64 userdata = 0);

	hacd::HaI32 m_incidentVertex;
	hacd::HaI32 m_incidentFace;
	hacd::HaU64 m_userData;
	dgEdge* m_next;
	dgEdge* m_prev;
	dgEdge* m_twin;
	hacd::HaI32 m_mark;
};


class dgPolyhedra: public dgTree <dgEdge, dgEdgeKey>, public hacd::UserAllocated
{
	public:

	struct dgPairKey
	{
		dgPairKey ();
		dgPairKey (hacd::HaI64 val);
		dgPairKey (hacd::HaI32 v0, hacd::HaI32 v1);
		hacd::HaI64 GetVal () const; 
		hacd::HaI32 GetLowKey () const;
		hacd::HaI32 GetHighKey () const;

		private:
		hacd::HaU64 m_key;
	};

	dgPolyhedra (void);
	dgPolyhedra (const dgPolyhedra &polyhedra);
	virtual ~dgPolyhedra();

	void BeginFace();
	dgEdge* AddFace (hacd::HaI32 v0, hacd::HaI32 v1, hacd::HaI32 v2);
	dgEdge* AddFace (hacd::HaI32 count, const hacd::HaI32* const index);
	dgEdge* AddFace (hacd::HaI32 count, const hacd::HaI32* const index, const hacd::HaI64* const userdata);
	void EndFace ();
	void DeleteFace(dgEdge* const edge);

	hacd::HaI32 GetFaceCount() const;
	hacd::HaI32 GetEdgeCount() const;
	hacd::HaI32 GetLastVertexIndex() const;

	hacd::HaI32 IncLRU() const;
	void SetLRU(hacd::HaI32 lru) const;

	dgEdge* FindEdge (hacd::HaI32 v0, hacd::HaI32 v1) const;
	dgTreeNode* FindEdgeNode (hacd::HaI32 v0, hacd::HaI32 v1) const;

	dgEdge* AddHalfEdge (hacd::HaI32 v0, hacd::HaI32 v1);
	void DeleteEdge (dgEdge* const edge);
	void DeleteEdge (hacd::HaI32 v0, hacd::HaI32 v1);
	
	bool FlipEdge (dgEdge* const edge);
	dgEdge* SpliteEdge (hacd::HaI32 newIndex, dgEdge* const edge);
	dgBigVector FaceNormal (dgEdge* const face, const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes) const;

	void BeginConectedSurface() const;
	bool GetConectedSurface (dgPolyhedra &polyhedra) const;
	void EndConectedSurface() const;

	dgSphere CalculateSphere (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes, const dgMatrix* const basis = NULL) const;
	void ChangeEdgeIncidentVertex (dgEdge* const edge, hacd::HaI32 newIndex);	
	void DeleteDegenerateFaces (const hacd::HaF64* const pool, hacd::HaI32 dstStrideInBytes, hacd::HaF64 minArea);

	void Optimize (const hacd::HaF64* const pool, hacd::HaI32 strideInBytes, hacd::HaF64 tol);
	void Triangulate (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes, dgPolyhedra* const leftOversOut);
	void ConvexPartition (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes, dgPolyhedra* const leftOversOut);

/*
	bool SanityCheck() const;


	// create an edge and add it to the tree. 
	// the edge is not linked to the existing edge list
	

	// create a complete edge and add it to the tree
	//	the new edge is linked to the existing edge list
//	dgEdge* AddEdge (hacd::HaI32 v0, hacd::HaI32 v1);

	void DeleteEdge (hacd::HaI32 v0, hacd::HaI32 v1);
	void DeleteEdge (dgEdge* const edge);

	void DeleteAllFace();

	

  	hacd::HaI32 GetMaxIndex() const;
  	
	hacd::HaI32 GetUnboundedFaceCount() const;
	
	dgBigVector BigFaceNormal (dgEdge* const face, const hacd::HaF64* const pool, hacd::HaI32 strideInBytes) const;


	
	
	dgEdge* SpliteEdgeAndTriangulate (hacd::HaI32 newIndex, dgEdge* const edge);

	dgEdge* FindVertexNode (hacd::HaI32 v0) const;
	
	hacd::HaI32 PackVertex (hacd::HaF64* const destArray, const hacd::HaF64* const unpackArray, hacd::HaI32 strideInBytes);
	void DeleteOverlapingFaces (const hacd::HaF64* const pool, hacd::HaI32 strideInBytes, hacd::HaF64 distTol);
	void InvertWinding ();

	// find edges edge shared by two or more non adjacent faces
	// this make impossible to triangulate the polyhedra
	void GetBadEdges (dgList<dgEdge*>& faceList, const hacd::HaF64* const pool, hacd::HaI32 strideInBytes) const; 

	
	void GetCoplanarFaces (dgList<dgEdge*>& faceList, dgEdge* startFace, const hacd::HaF64* const pool, hacd::HaI32 hisStrideInBytes, hacd::HaF64 normalDeviation) const;
	void GetOpenFaces (dgList<dgEdge*>& faceList) const;
	void CollapseDegenerateFaces (dgPolyhedraDescriptor &desc, const hacd::HaF64* const pool, hacd::HaI32 strideInBytes, hacd::HaF64 area);
	


	// this function assume the mesh is a legal mesh;
	hacd::HaI32 TriangleList (hacd::HaU32 outputBuffer[], hacd::HaI32 maxBufferSize, hacd::HaI32 vertexCacheSize = 12) const;
	void SwapInfo (dgPolyhedra& source);


*/

	// this function ayend to create a better triangulation of a mesh
	// by first calling the calling quadrangular and then triangulate 
	// all quad strips.
	// this function assume the mesh is a legal mesh;
	// note1: recommend a call to Triangulate or OptimalTriangulation 
	//			 before using this function
	// note2: a index set to 0xffffffff indicate a run start
	// return index count
	//	hacd::HaI32 TriangleStrips (hacd::HaU32 outputBuffer[], hacd::HaI32 maxBufferSize, hacd::HaI32 vertexCacheSize = 12) const;
	//	void OptimalTriangulation (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes);
	//	void CombineOpenFaces (const hacd::HaF64* const pool, hacd::HaI32 strideInBytes, hacd::HaF64 distTol);
	//	bool TriangulateFace (dgEdge* const face, const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes, dgBigVector& normalOut);
	//	void OptimizeTriangulation (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes);
	//	void Quadrangulate (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes);
	//	dgEdge* GetBadEdge (dgList<dgEdge*>& faceList  const hacd::HaF64* const pool, hacd::HaI32 strideInBytes) const; 

	private:
	
	void RefineTriangulation (const hacd::HaF64* const vertex, hacd::HaI32 stride);
	void RefineTriangulation (const hacd::HaF64* const vertex, hacd::HaI32 stride, dgBigVector* const normal, hacd::HaI32 perimeterCount, dgEdge** const perimeter);
	void OptimizeTriangulation (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes);
	void MarkAdjacentCoplanarFaces (dgPolyhedra& polyhedraOut, dgEdge* const face, const hacd::HaF64* const pool, hacd::HaI32 strideInBytes);
	dgEdge* FindEarTip (dgEdge* const face, const hacd::HaF64* const pool, hacd::HaI32 stride, dgDownHeap<dgEdge*, hacd::HaF64>& heap, const dgBigVector &normal) const;
	dgEdge* TriangulateFace (dgEdge* face, const hacd::HaF64* const pool, hacd::HaI32 stride, dgDownHeap<dgEdge*, hacd::HaF64>& heap, dgBigVector* const faceNormalOut);
	
	hacd::HaF64 EdgePenalty (const dgBigVector* const pool, dgEdge* const edge) const;

	mutable hacd::HaI32 m_baseMark;
	mutable hacd::HaI32 m_edgeMark;
	mutable hacd::HaI32 m_faceSecuence;

	friend class dgPolyhedraDescriptor;
	
};



inline dgEdge::dgEdge ()
{
}

inline dgEdge::dgEdge (hacd::HaI32 vertex, hacd::HaI32 face, hacd::HaU64 userdata)
	:m_incidentVertex(vertex)
	,m_incidentFace(face)
	,m_userData(userdata)
	,m_next(NULL)
	,m_prev(NULL)
	,m_twin(NULL)
	,m_mark(0)
{
}


inline dgPolyhedra::dgPairKey::dgPairKey ()
{
}

inline dgPolyhedra::dgPairKey::dgPairKey (hacd::HaI64 val)
	:m_key(hacd::HaU64 (val))
{
}

inline dgPolyhedra::dgPairKey::dgPairKey (hacd::HaI32 v0, hacd::HaI32 v1)
	:m_key (hacd::HaU64 ((hacd::HaI64 (v0) << 32) | v1))
{
}

inline hacd::HaI64 dgPolyhedra::dgPairKey::GetVal () const 
{
	return hacd::HaI64 (m_key);
}

inline hacd::HaI32 dgPolyhedra::dgPairKey::GetLowKey () const 
{
	return hacd::HaI32 (m_key>>32);
}

inline hacd::HaI32 dgPolyhedra::dgPairKey::GetHighKey () const 
{
	return hacd::HaI32 (m_key & 0xffffffff);
}

inline void dgPolyhedra::BeginFace ()
{
}

inline dgEdge* dgPolyhedra::AddFace (hacd::HaI32 count, const hacd::HaI32* const index) 
{
	return AddFace (count, index, NULL);
}

inline dgEdge* dgPolyhedra::AddFace (hacd::HaI32 v0, hacd::HaI32 v1, hacd::HaI32 v2)
{
	hacd::HaI32 vertex[3];

	vertex [0] = v0;
	vertex [1] = v1;
	vertex [2] = v2;
	return AddFace (3, vertex, NULL);
}

inline hacd::HaI32 dgPolyhedra::GetEdgeCount() const
{
#ifdef _DEBUG
	hacd::HaI32 edgeCount = 0;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) {
		edgeCount ++;
	}
	HACD_ASSERT (edgeCount == GetCount());;
#endif
	return GetCount();
}

inline hacd::HaI32 dgPolyhedra::GetLastVertexIndex() const
{
	hacd::HaI32 maxVertexIndex = -1;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) {
		const dgEdge* const edge = &(*iter);
		if (edge->m_incidentVertex > maxVertexIndex) {
			maxVertexIndex = edge->m_incidentVertex;
		}
	}
	return maxVertexIndex + 1;
}


inline hacd::HaI32 dgPolyhedra::IncLRU() const
{	
	m_edgeMark ++;
	HACD_ASSERT (m_edgeMark < 0x7fffffff);
	return m_edgeMark;
}

inline void dgPolyhedra::SetLRU(hacd::HaI32 lru) const
{
	if (lru > m_edgeMark) {
		m_edgeMark = lru;
	}
}

inline void dgPolyhedra::BeginConectedSurface() const
{
	m_baseMark = IncLRU();
}

inline void dgPolyhedra::EndConectedSurface() const
{
}


inline dgPolyhedra::dgTreeNode* dgPolyhedra::FindEdgeNode (hacd::HaI32 i0, hacd::HaI32 i1) const
{
	dgPairKey key (i0, i1);
	return Find (key.GetVal());
}


inline dgEdge *dgPolyhedra::FindEdge (hacd::HaI32 i0, hacd::HaI32 i1) const
{
	//	dgTreeNode *node;
	//	dgPairKey key (i0, i1);
	//	node = Find (key.GetVal());
	//	return node ? &node->GetInfo() : NULL;
	dgTreeNode* const node = FindEdgeNode (i0, i1);
	return node ? &node->GetInfo() : NULL;
}

inline void dgPolyhedra::DeleteEdge (hacd::HaI32 v0, hacd::HaI32 v1)
{
	dgPairKey pairKey (v0, v1);
	dgTreeNode* const node = Find(pairKey.GetVal());
	dgEdge* const edge = node ? &node->GetInfo() : NULL;
	if (!edge) {
		return;
	}
	DeleteEdge (edge);
}


#endif

