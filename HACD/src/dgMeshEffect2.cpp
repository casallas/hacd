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

#include "dgMeshEffect.h"
#include "dgConvexHull3d.h"
#include "dgStack.h"
#include <string.h>

#pragma warning(disable:4100)

// create a convex hull
dgMeshEffect::dgMeshEffect (const hacd::HaF64* const vertexCloud, hacd::HaI32 count, hacd::HaI32 strideInByte, hacd::HaF64 distTol)
	:dgPolyhedra()
{
	Init(true);
	if (count >= 4) {
		dgConvexHull3d convexHull (vertexCloud, strideInByte, count, distTol);
		if (convexHull.GetCount()) {

			hacd::HaI32 vertexCount = convexHull.GetVertexCount();
			dgStack<dgVector> pointsPool (convexHull.GetVertexCount());
			dgVector* const points = &pointsPool[0];
			for (hacd::HaI32 i = 0; i < vertexCount; i ++) {
				points[i] = convexHull.GetVertex(i);
			}
			dgVector uv(hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
			dgVector normal (hacd::HaF32 (0.0f), hacd::HaF32 (1.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));

			hacd::HaI32 triangleCount = convexHull.GetCount();
			dgStack<hacd::HaI32> faceCountPool (triangleCount);
			dgStack<hacd::HaI32> materialsPool (triangleCount);
			dgStack<hacd::HaI32> vertexIndexListPool (triangleCount * 3);
			dgStack<hacd::HaI32> normalIndexListPool (triangleCount * 3);


			memset (&materialsPool[0], 0, triangleCount * sizeof (hacd::HaI32));
			memset (&normalIndexListPool[0], 0, 3 * triangleCount * sizeof (hacd::HaI32));

			hacd::HaI32 index = 0;
			hacd::HaI32* const faceCount = &faceCountPool[0];
			hacd::HaI32* const vertexIndexList = &vertexIndexListPool[0];
			for (dgConvexHull3d::dgListNode* faceNode = convexHull.GetFirst(); faceNode; faceNode = faceNode->GetNext()) {
				dgConvexHull3DFace& face = faceNode->GetInfo();
				faceCount[index] = 3;
				vertexIndexList[index * 3 + 0] = face.m_index[0]; 
				vertexIndexList[index * 3 + 1] = face.m_index[1]; 
				vertexIndexList[index * 3 + 2] = face.m_index[2]; 
				index ++;
			}

			BuildFromVertexListIndexList(triangleCount, faceCount, &materialsPool[0], 
				&points[0].m_x, sizeof (dgVector), vertexIndexList,
				&normal.m_x, sizeof (dgVector), &normalIndexListPool[0],
				&uv.m_x, sizeof (dgVector), &normalIndexListPool[0],
				&uv.m_x, sizeof (dgVector), &normalIndexListPool[0]);
		}
	}
}


#if 0

class Tetrahedralization: public dgDelaunayTetrahedralization
{
	class dgIndexMapPair
	{
		public:
		hacd::HaI32 m_meshIndex;
		hacd::HaI32 m_convexIndex;
	};

	class dgMissingEdges: public dgList<dgPolyhedra::dgTreeNode*> 
	{	
		public:
		dgMissingEdges ()
			:dgList<dgPolyhedra::dgTreeNode*> ()
		{
		}
		~dgMissingEdges()
		{
		}
	};

	class dgEdgeSharedTetras: public dgList<dgListNode*>
	{
		public:
		dgEdgeSharedTetras(const dgEdgeSharedTetras& copy)
			:dgList(copy.GetAllocator())
		{
		}

		dgEdgeSharedTetras()
			:dgList()
		{
		}

		~dgEdgeSharedTetras ()
		{
		}
	};

	class dgEdgeMap: public dgTree<dgEdgeSharedTetras, hacd::HaU64>
	{
		public:
		dgEdgeMap()
			:dgTree<dgEdgeSharedTetras, hacd::HaU64>()
		{
		}

		~dgEdgeMap()
		{
			while(GetRoot()) {
				dgEdgeSharedTetras& header = GetRoot()->GetInfo();
				header.RemoveAll();
				Remove(GetRoot());
			}
		}
	};

	class dgVertexMap: public dgTree<dgEdgeSharedTetras, hacd::HaI32>
	{
		public:
		dgVertexMap()
			:dgTree<dgEdgeSharedTetras, hacd::HaI32>()
		{
		}

		~dgVertexMap()
		{
			while(GetRoot()) {
				dgEdgeSharedTetras& header = GetRoot()->GetInfo();
				header.RemoveAll();
				Remove(GetRoot());
			}
		}
	};


/*
#ifdef _DEBUG
	class dgEdgeFaceKey
	{
	public:
		dgEdgeFaceKey ()
		{}

		dgEdgeFaceKey (hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2)
		{
			m_index[0] = i0;
			m_index[1] = i1;
			m_index[2] = i2;
			while ((m_index[0] > m_index[1]) || (m_index[0] > m_index[2])) {
				i0 = m_index[0];
				m_index[0] = m_index[1];
				m_index[1] = m_index[2];
				m_index[2] = i0;
			}
		}

		hacd::HaI32 Compared (const dgEdgeFaceKey& key) const 
		{
			for (hacd::HaI32 i = 0; i < 3; i ++) {
				if (m_index[i] < key.m_index[i]) {
					return -1;
				} else if (m_index[i] > key.m_index[i]) {
					return 1;
				}
			}
			return 0;
		}


		bool operator < (const dgEdgeFaceKey& key) const 
		{
			return (Compared (key) < 0);
		}

		bool operator > (const dgEdgeFaceKey& key) const 
		{
			return (Compared (key) > 0);
		}


		hacd::HaI32 m_index[3];

	};


	class dgFaceKeyMap: public dgTree<dgListNode*, dgEdgeFaceKey>
	{
	public:
		dgFaceKeyMap()
			:dgTree<dgListNode*, dgEdgeFaceKey>()
		{

		}
	};
#endif

*/
public:
	Tetrahedralization (dgMeshEffect& mesh)
		:dgDelaunayTetrahedralization (mesh.GetAllocator(), mesh.GetVertexPool(), mesh.GetVertexCount(), sizeof (dgVector), 0.0f),
		m_mesh (&mesh),
		m_edgeMap (mesh.GetAllocator()), 
		m_vertexMap (mesh.GetAllocator()), 
		m_missinEdges(mesh.GetAllocator()), 
		m_indexMap (mesh.GetVertexCount() * 8 + 2048, mesh.GetAllocator())
	{
		if (GetCount()) {

			#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
			hacd::HaU32 controlWorld = dgControlFP (0xffffffff, 0);
			dgControlFP (_PC_53, _MCW_PC);
			#endif

			// add every edge of each tetrahedral to a edge list
			BuildTetrahedraEdgeListAndVertexList ();

			// make a index map to quickly find vertex mapping form the mesh to the delaunay tetrahedron
			CreateIndexMap ();

			// Insert all missing edge in mesh as a new into the tetrahedral list
			RecoverEdges ();

			// Recover the solid mesh from the delaunay tetrahedron	
			RecoverFaces ();

			#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
			dgControlFP (controlWorld, _MCW_PC);
			#endif
		}
	}

	~Tetrahedralization()
	{
	}

	hacd::HaU64 GetKey (hacd::HaI32 i0, hacd::HaI32 i1) const
	{
		return (i1 > i0) ?  (hacd::HaU64 (i1) << 32) + i0 : (hacd::HaU64 (i0) << 32) + i1;
	}

	void InsertNewEdgeNode (hacd::HaI32 i0, hacd::HaI32 i1, dgListNode* const node)
	{
		hacd::HaU64 key = GetKey (i1, i0);

		dgEdgeMap::dgTreeNode* edgeNode = m_edgeMap.Find(key);
		if (!edgeNode) {
			dgEdgeSharedTetras tmp (GetAllocator());
			edgeNode = m_edgeMap.Insert(tmp, key);
		}
		dgEdgeSharedTetras& header = edgeNode->GetInfo();

		#ifdef _DEBUG
			for (dgEdgeSharedTetras::dgListNode* ptr = header.GetFirst(); ptr; ptr = ptr->GetNext()) {
				HACD_ASSERT (ptr->GetInfo() != node);
			}
		#endif

		header.Append(node);
	}

	void RemoveEdgeNode(hacd::HaI32 i0, hacd::HaI32 i1, dgListNode* const node)
	{
		hacd::HaU64 key = GetKey (i0, i1);

		dgEdgeMap::dgTreeNode* const edgeNode = m_edgeMap.Find(key);
		if (edgeNode) {
			dgEdgeSharedTetras& header = edgeNode->GetInfo();
			for (dgEdgeSharedTetras::dgListNode* ptr = header.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dgListNode* const me = ptr->GetInfo();
				if (me == node)  {
					header.Remove(ptr);
					if (!header.GetCount()) {
						m_edgeMap.Remove(edgeNode);
					}

//					hacd::HaI32 index0 = GetVertexIndex(i0);
//					hacd::HaI32 index1 = GetVertexIndex(i1);
//					dgPolyhedra::dgTreeNode* const edgeNode = m_mesh->FindEdgeNode(index0, index1);
//					if(edgeNode) {
//						m_missinEdges.Append(edgeNode);
//					}

					break;
				}
			}
		}
	}


	void InsertNewVertexNode (hacd::HaI32 index, dgListNode* const node)
	{
		dgVertexMap::dgTreeNode* vertexNode = m_vertexMap.Find(index);
		if (!vertexNode) {
			dgEdgeSharedTetras tmp (GetAllocator());
			vertexNode = m_vertexMap.Insert(tmp, index);
		}
		dgEdgeSharedTetras& header = vertexNode->GetInfo();

		#ifdef _DEBUG
			for (dgEdgeSharedTetras::dgListNode* ptr = header.GetFirst(); ptr; ptr = ptr->GetNext()) {
				HACD_ASSERT (ptr->GetInfo() != node);
			}
		#endif

		header.Append(node);
	}

	void RemoveNewVertexNode (hacd::HaI32 index, dgListNode* const node)
	{
		dgVertexMap::dgTreeNode* vertexNode = m_vertexMap.Find(index);
		HACD_ASSERT (vertexNode);
		dgEdgeSharedTetras& header = vertexNode->GetInfo();

		for (dgEdgeSharedTetras::dgListNode* ptr = header.GetFirst(); ptr; ptr = ptr->GetNext()) {
			if (ptr->GetInfo() == node) {
				header.Remove(node);
				break;
			}
		}
		HACD_ASSERT (header.GetCount());

	}
	

	void AddEdgesAndFaces(dgListNode* const node)
	{
		dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
		if (GetTetraVolume (tetra) < 0.0f) {
			const dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[0];
			for (hacd::HaI32 i = 0; i < 3; i ++) {
				hacd::HaI32 i0 = face.m_otherVertex;
				hacd::HaI32 i1 = face.m_index[i];
				InsertNewEdgeNode (i0, i1, node);
			}

			hacd::HaI32 i0 = face.m_index[2];
			for (hacd::HaI32 i = 0; i < 3; i ++) {
				hacd::HaI32 i1 = face.m_index[i];
				InsertNewEdgeNode (i0, i1, node);
				InsertNewVertexNode (i0, node);
				i0 = i1;
			}
			InsertNewVertexNode (face.m_otherVertex, node);
		}
	}

	void RemoveEdgesAndFaces (dgListNode* const node)
	{
		dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
		const dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[0];
		for (hacd::HaI32 i = 0; i < 3; i ++) {
			hacd::HaI32 i0 = face.m_otherVertex;
			hacd::HaI32 i1 = face.m_index[i];
			RemoveEdgeNode(i0, i1, node);
		}

		hacd::HaI32 i0 = face.m_index[2];
		for (hacd::HaI32 i = 0; i < 3; i ++) {
			hacd::HaI32 i1 = face.m_index[i];
			RemoveEdgeNode(i0, i1, node);
			RemoveNewVertexNode(i0, node);
			i0 = i1;
		}

		RemoveNewVertexNode(face.m_otherVertex, node);
	}


	dgListNode* AddFace (hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2, hacd::HaI32 i3)
	{
		dgListNode* const faceNode = dgDelaunayTetrahedralization::AddFace(i0, i1, i2, i3);
		AddEdgesAndFaces (faceNode);
		return faceNode;
	}


	void DeleteFace (dgListNode* const node) 
	{
		RemoveEdgesAndFaces (node);

		dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
		for (hacd::HaI32 i= 0; i < 4; i ++) {
			const dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[i];
			dgListNode* const twinNode = face.m_twin;
			if (twinNode) {
				dgConvexHull4dTetraherum* const twinTetra = &twinNode->GetInfo();
				for (hacd::HaI32 i = 0; i < 4; i ++) {
					if (twinTetra->m_faces[i].m_twin == node) {
						twinTetra->m_faces[i].m_twin = NULL;
					}
				}
			}
		}

		dgDelaunayTetrahedralization::DeleteFace(node);
	}



	void BuildTetrahedraEdgeListAndVertexList () 
	{
		for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			AddEdgesAndFaces (node);
		}
	}


	static hacd::HaI32 ConvexCompareIndex(const dgIndexMapPair* const  A, const dgIndexMapPair* const B, void* const context)
	{
		if (A->m_meshIndex > B->m_meshIndex) {
			return 1;
		} else if (A->m_meshIndex < B->m_meshIndex) {
			return -1;
		}
		return 0;
	}

	void CreateIndexMap ()
	{
		// make a index map to quickly find vertex mapping form the mesh to the delaunay tetrahedron
		m_indexMap[GetVertexCount()].m_meshIndex = 0;
		dgIndexMapPair* const indexMap = &m_indexMap[0];
		for (hacd::HaI32 i = 0; i < GetVertexCount(); i ++) {
			indexMap[i].m_convexIndex = i;
			indexMap[i].m_meshIndex = GetVertexIndex(i);
		}
		dgSort(indexMap, GetVertexCount(), ConvexCompareIndex);
	}


	bool SanityPointInTetra (dgConvexHull4dTetraherum* const tetra, const dgBigVector& vertex) const 
	{

		for (hacd::HaI32 i = 0; i < 4; i ++) {
			const dgBigVector& p0 = m_points[tetra->m_faces[i].m_index[0]];
			const dgBigVector& p1 = m_points[tetra->m_faces[i].m_index[1]];
			const dgBigVector& p2 = m_points[tetra->m_faces[i].m_index[2]];
			dgBigPlane plane (p0, p1, p2);
			hacd::HaF64 dist = plane.Evalue(vertex);
			if (dist > hacd::HaF64 (1.0e-12f)) {
				return false;
			}
		}
		return true;
	}

	hacd::HaI32 ReplaceFaceNodes (dgListNode* const faceNode, hacd::HaI32 faceIndex, const dgBigVector& vertex)
	{
		dgConvexHull4dTetraherum* const tetra = &faceNode->GetInfo();
		dgListNode* const neighborghNode = tetra->m_faces[faceIndex].m_twin;
		HACD_ASSERT (neighborghNode);

		dgConvexHull4dTetraherum* const neighborghTetra = &neighborghNode->GetInfo();

		hacd::HaI32 vertexIndex = m_count;
		m_points[vertexIndex] = vertex;
		m_points[vertexIndex].m_index = vertexIndex;
		m_count ++;

		HACD_ASSERT (SanityPointInTetra (tetra, vertex));
		HACD_ASSERT (SanityPointInTetra (neighborghTetra, vertex));

		hacd::HaI32 mark = IncMark();
		tetra->SetMark(mark);
		neighborghTetra->SetMark(mark);

		hacd::HaI32 deletedCount = 2;
		dgListNode* deletedNodes[2];
		deletedNodes[0] = faceNode;
		deletedNodes[1] = neighborghNode;

		hacd::HaI32 perimeterCount = 0;
		dgListNode* perimeter[16];
		for (hacd::HaI32 i = 0; i < deletedCount; i ++) {
			dgListNode* const deleteTetraNode = deletedNodes[i];

			dgConvexHull4dTetraherum* const deletedTetra = &deleteTetraNode->GetInfo();
			HACD_ASSERT (deletedTetra->GetMark() == mark);

			for (hacd::HaI32 i = 0; i < 4; i ++) {
				dgListNode* const twinNode = deletedTetra->m_faces[i].m_twin;
				dgConvexHull4dTetraherum* const twinTetra = &twinNode->GetInfo();
				HACD_ASSERT (twinTetra);

				if (twinTetra->GetMark() != mark) {
					hacd::HaI32 index = 0;
					for (index = 0; index < perimeterCount; index ++) {
						if (perimeter[index] == twinNode) {
							break;
						}
					}
					if (index == perimeterCount) {
						perimeter[perimeterCount] = twinNode;
						perimeterCount ++;
					}
				}
				deletedTetra->m_faces[i].m_twin = NULL;
			}
		}

		hacd::HaI32 coneListCount = 0;
		dgListNode* coneList[32];
		for (hacd::HaI32 i = 0; i < perimeterCount; i ++) {
			dgListNode* const perimeterNode = perimeter[i];
			dgConvexHull4dTetraherum* const perimeterTetra = &perimeterNode->GetInfo();

			for (hacd::HaI32 i = 0; i < 4; i ++) {
				dgConvexHull4dTetraherum::dgTetrahedrumFace* const perimeterFace = &perimeterTetra->m_faces[i];
				if (perimeterFace->m_twin->GetInfo().GetMark() == mark) {

					dgListNode* const newNode = AddFace (vertexIndex, perimeterFace->m_index[0], perimeterFace->m_index[1], perimeterFace->m_index[2]);

					dgConvexHull4dTetraherum* const newTetra = &newNode->GetInfo();
					newTetra->m_faces[2].m_twin = perimeterNode;
					perimeterFace->m_twin = newNode;
					coneList[coneListCount] = newNode;
					coneListCount ++;
				}
			}
		}


		for (int i = 0; i < (coneListCount - 1); i ++) {
			dgListNode* const coneNodeA = coneList[i];
			for (hacd::HaI32 j = i + 1; j < coneListCount; j ++) {
				dgListNode* const coneNodeB = coneList[j];
				LinkSibling (coneNodeA, coneNodeB);
			}
		}

		for (hacd::HaI32 i = 0; i < deletedCount; i ++) {
			//dgListNode* const node = deleteNode->GetInfo();
			dgListNode* const deleteTetraNode = deletedNodes[i];
			DeleteFace (deleteTetraNode); 
		}

		return vertexIndex;
	}



	void RecoverEdges ()
	{
		// split every missing edge at the center and add the two half to the triangulation
		// keep doing it until all edge are present in the triangulation.

		hacd::HaI32 mark = m_mesh->IncLRU();

		// create a list all all the edge that are in the mesh but that do not appear in the delaunay tetrahedron 
		const dgIndexMapPair* const indexMap = &m_indexMap[0];
		dgPolyhedra::Iterator iter (*m_mesh);

		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark) {
				edge->m_mark = mark;
				edge->m_twin->m_mark = mark;

				hacd::HaI32 i0 = indexMap[edge->m_incidentVertex].m_convexIndex;
				hacd::HaI32 i1 = indexMap[edge->m_twin->m_incidentVertex].m_convexIndex;
				hacd::HaU64 key = GetKey (i0, i1);
				dgEdgeMap::dgTreeNode* const edgeNode = m_edgeMap.Find(key);
				if (!edgeNode) {
					m_missinEdges.Append(iter.GetNode());
				}
			}
		}

//m_missinEdges.Remove(m_missinEdges.GetLast());

		while (m_missinEdges.GetCount()){
			dgIndexMapPair* const indexMap = &m_indexMap[0];

			dgMissingEdges::dgListNode* missingEdgeNode = m_missinEdges.GetFirst();
			dgEdge* missingEdge = &missingEdgeNode->GetInfo()->GetInfo();

			hacd::HaI32 k0 = missingEdge->m_incidentVertex;
			hacd::HaI32 k1 = missingEdge->m_twin->m_incidentVertex;
			hacd::HaI32 i0 = indexMap[k0].m_convexIndex;
			hacd::HaI32 i1 = indexMap[k1].m_convexIndex;

			m_missinEdges.Remove(missingEdgeNode);
			hacd::HaU64 key = GetKey (i0, i1);
			if (!m_edgeMap.Find(key)) {
				dgVertexMap::dgTreeNode* const vertexNode = m_vertexMap.Find(i0);
				HACD_ASSERT (vertexNode);
				const dgEdgeSharedTetras& tetraMap = vertexNode->GetInfo();

				const dgBigVector& p0 = GetVertex(i0);
				const dgBigVector& p1 = GetVertex(i1);
				bool edgeFound = false;
				for (dgEdgeSharedTetras::dgListNode* node = tetraMap.GetFirst(); node; node = node->GetNext()) {
					dgListNode* const tetraNode = node->GetInfo();
					dgConvexHull4dTetraherum* const tetra = &tetraNode->GetInfo();
					hacd::HaI32 faceIndex = -1;
					for (hacd::HaI32 i = 0; i < 4; i ++) {
						if (tetra->m_faces[i].m_otherVertex == i0) {
							faceIndex = i;	
						}
					}
					HACD_ASSERT (faceIndex != -1);

					const dgBigVector& A = GetVertex(tetra->m_faces[faceIndex].m_index[0]);
					const dgBigVector& B = GetVertex(tetra->m_faces[faceIndex].m_index[1]);
					const dgBigVector& C = GetVertex(tetra->m_faces[faceIndex].m_index[2]);
					dgBigVector baricentric (LineTriangleIntersection (p0, p1, A, B, C));
					if (baricentric.m_w == hacd::HaF64 (0.0f)) {
						HACD_ASSERT ((baricentric.m_x > hacd::HaF64 (0.0f)) && (baricentric.m_y > hacd::HaF64 (0.0f)) && (baricentric.m_z > hacd::HaF64 (0.0f)));
						dgBigVector point (A.Scale4 (baricentric.m_x) + B.Scale4 (baricentric.m_y) + C.Scale4 (baricentric.m_z));
						hacd::HaI32 index = ReplaceFaceNodes(tetraNode, faceIndex, point);
						

						dgBigVector pp0 (point - p0);
						dgBigVector p1p0 (p1 - p0);
						hacd::HaF64 spliteParam = (pp0 % p1p0) / (p1p0 % p1p0);
						dgEdge* const newEdge = m_mesh->InsertEdgeVertex (missingEdge, hacd::HaF32 (spliteParam));

						indexMap[newEdge->m_twin->m_incidentVertex].m_convexIndex = index;

						i0 = indexMap[newEdge->m_next->m_incidentVertex].m_convexIndex;
						i1 = indexMap[newEdge->m_next->m_twin->m_incidentVertex].m_convexIndex;
						key = GetKey (i0, i1);
						if (!m_edgeMap.Find(key)) {
							hacd::HaI32 index0 = GetVertexIndex(i0);
							hacd::HaI32 index1 = GetVertexIndex(i1);
							dgPolyhedra::dgTreeNode* const edgeNode = m_mesh->FindEdgeNode(index0, index1);
							HACD_ASSERT (edgeNode);
							m_missinEdges.Addtop(edgeNode);
						}


						edgeFound = true;
						break;
					}
				}
				HACD_ASSERT (edgeFound);
			} 
		}
	}

/*
	void RemoveDegeneratedTetras ()
	{
		hacd::HaI32 mark = m_mesh->IncLRU(); 
		dgPolyhedra::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const faceEdge = &iter.GetNode()->GetInfo();
			hacd::HaI32 count = 0; 
			dgEdge* ptr = faceEdge;
			do {
				count ++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != faceEdge);

			if (count > 3) {
				dgEdge* ptr = faceEdge;
				do {
					hacd::HaI32 k0 = m_indexMap[ptr->m_incidentVertex].m_convexIndex;
					hacd::HaI32 k1 = m_indexMap[ptr->m_next->m_incidentVertex].m_convexIndex;
					hacd::HaU64 key = GetKey (k0, k1);
					dgEdgeMap::dgTreeNode* const edgeNode = m_edgeMap.Find(key);
					if (edgeNode) {
						dgEdgeSharedTetras& header = edgeNode->GetInfo();
						for (dgEdgeSharedTetras::dgListNode* ptr1 = header.GetFirst(); ptr1; ptr1 = ptr1->GetNext()) {
							dgListNode* const tetraNode = ptr1->GetInfo();
							dgConvexHull4dTetraherum* const tetra = &tetraNode->GetInfo();
							const dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[0];
							hacd::HaI32 index[4];
							index[0] = GetVertexIndex(face.m_index[0]);
							index[1] = GetVertexIndex(face.m_index[1]);
							index[2] = GetVertexIndex(face.m_index[2]);
							index[3] = GetVertexIndex(face.m_otherVertex);

							hacd::HaI32 duplicates = 0;
							dgEdge* ptr3 = faceEdge;
							do {
								for (hacd::HaI32 i = 0; i < 4; i ++) {
									duplicates += (ptr3->m_incidentVertex == index[i]) ? 1 : 0;
								}
								ptr3 = ptr3->m_next;
							} while (ptr3 != faceEdge);
							if (duplicates > 3) {
								DeleteFace(tetraNode);
								break;
							}
						}
					}

					ptr = ptr->m_next;
				} while (ptr != faceEdge);
			}
		}
	}
*/

	bool MatchFace (dgMeshEffect& mesh, dgEdge* const faceEdge, hacd::HaI32 tetraMark) const
	{
		hacd::HaI32 k0 = m_indexMap[faceEdge->m_incidentVertex].m_convexIndex;
		hacd::HaI32 k1 = m_indexMap[faceEdge->m_next->m_incidentVertex].m_convexIndex;
		hacd::HaU64 key = GetKey (k0, k1);
		dgEdgeMap::dgTreeNode* const edgeNode = m_edgeMap.Find(key);

		HACD_ASSERT (edgeNode);
		dgEdgeSharedTetras& header = edgeNode->GetInfo();
		for (dgEdgeSharedTetras::dgListNode* ptr = header.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dgListNode* const tetraNode = ptr->GetInfo();
			dgConvexHull4dTetraherum* const tetra = &tetraNode->GetInfo();
			for (hacd::HaI32 i = 0; i < 4; i ++) {
				dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[i];
				hacd::HaI32 i0 = face.m_index[0];
				hacd::HaI32 i1 = face.m_index[1];
				hacd::HaI32 i2 = face.m_index[2];

				if (((i0 == k0) && (i1 == k1)) || ((i1 == k0) && (i2 == k1)) || ((i2 == k0) && (i0 == k1))) {
					hacd::HaI32 index[3];

					index[0] = GetVertexIndex (i0);
					index[1] = GetVertexIndex (i1);
					index[2] = GetVertexIndex (i2);
					while (index[0] != faceEdge->m_incidentVertex) {
						hacd::HaI32 tmp = index[0];
						index[0] = index[1];
						index[1] = index[2];
						index[2] = tmp;
					}
					HACD_ASSERT (index[0] == faceEdge->m_incidentVertex);
					HACD_ASSERT (index[1] == faceEdge->m_next->m_incidentVertex);

					dgEdge* nextEdge = faceEdge->m_next->m_next;
					do {
						if (nextEdge->m_incidentVertex == index[2]) {
							break;
						}
						nextEdge = nextEdge->m_next;
					} while (nextEdge != faceEdge);

					if (nextEdge != faceEdge) {
						if (nextEdge->m_prev != faceEdge->m_next) {
							dgEdge* const edge = mesh.ConectVertex(faceEdge->m_next, nextEdge);
							HACD_ASSERT (edge);
							HACD_ASSERT (edge->m_next);
							HACD_ASSERT (edge->m_prev);
							HACD_ASSERT (edge->m_twin->m_next);
							HACD_ASSERT (edge->m_twin->m_prev);
							HACD_ASSERT (faceEdge->m_next == edge->m_twin);
						}
						if (nextEdge->m_next != faceEdge) {
#ifdef	_DEBUG
							dgEdge* const edge = mesh.ConectVertex(faceEdge, nextEdge);
							HACD_ASSERT (edge);
							HACD_ASSERT (edge->m_next);
							HACD_ASSERT (edge->m_prev);
							HACD_ASSERT (edge->m_twin->m_next);
							HACD_ASSERT (edge->m_twin->m_prev);
							HACD_ASSERT (faceEdge->m_prev == edge);
#else
							mesh.ConectVertex(faceEdge, nextEdge);
#endif

						}

						if (tetraMark != -1) {
							tetra->SetMark (tetraMark);
							face.m_twin = NULL;
						}
						return true;
					}
				}
			}
		}
		return false;
	}


	void RecoverFace (dgMeshEffect& mesh, dgEdge* const face, hacd::HaI32 faceMark, hacd::HaI32 tetraMark, dgTree<dgEdge*, dgEdge*>& edgeInconflict) const
	{
		hacd::HaI32 count = 0;
		hacd::HaI32 perimeterCount = 0;
		dgEdge* edgeArray[1024];
		dgEdge* perimterEdges[1024 + 1];

		dgEdge* ptr = face;
		do {
			edgeArray[count] = ptr;
			perimterEdges[count] = ptr;
			count ++;
			HACD_ASSERT (count < hacd::HaI32 (sizeof (edgeArray) / sizeof (edgeArray[0])));
			ptr = ptr->m_next;
		} while (ptr != face);
		perimeterCount = count;
		perimterEdges[count] = face;


		while (count) {
			count --;
			dgEdge* const triangleFace = edgeArray[count];
			bool state = MatchFace (mesh, triangleFace, tetraMark);
			if (state) {
				HACD_ASSERT (triangleFace == triangleFace->m_next->m_next->m_next);
				triangleFace->m_mark = faceMark;
				dgEdge* ptr = triangleFace->m_next; 
				do {
					ptr->m_mark = faceMark;
					for (hacd::HaI32 i = 0; i < count; i ++) {
						if (ptr == edgeArray[i]) {
							edgeArray[i] = edgeArray[count - 1];
							i --;
							count --;
							break;
						}
					}
					ptr = ptr->m_next;
				} while (ptr != triangleFace);
			}
		}

		HACD_ASSERT (count == 0);
		for (hacd::HaI32 i = 1; i <= perimeterCount; i ++) {
			dgEdge* const last = perimterEdges[i - 1];
			for (dgEdge* edge = perimterEdges[i]->m_prev; edge != last; edge = edge->m_twin->m_prev) {
				if (edge->m_mark != faceMark) {
					hacd::HaI32 index = 0;
					for (index = 0; index < count; index ++) {
						if ((edgeArray[index] == edge) || (edgeArray[index] == edge->m_twin)) {
							break;
						}
					}
					if (index == count) {
						edgeArray[count] = edge;
						count ++;
					}
				}
			}
		}

		if (count) {
			while (count) {
				count --;
				dgEdge* const triangleFace = edgeArray[count];
				bool state = MatchFace (mesh, triangleFace, tetraMark);
				if (state) {
					HACD_ASSERT (triangleFace == triangleFace->m_next->m_next->m_next);
					triangleFace->m_mark = faceMark;
					dgEdge* ptr = triangleFace->m_next; 
					do {
						ptr->m_mark = faceMark;
						for (hacd::HaI32 i = 0; i < count; i ++) {
							if (ptr == edgeArray[i]) {
								edgeArray[i] = edgeArray[count - 1];
								i --;
								count --;
								break;
							}
						}
						ptr = ptr->m_next;
					} while (ptr != triangleFace);
				}
			}

			HACD_ASSERT (count == 0);
			for (hacd::HaI32 i = 0; i < perimeterCount; i ++) {
				dgEdge* const edge = perimterEdges[i];
				if (edge->m_mark != faceMark) {
					dgEdge* const borderEdge = m_mesh->FindEdge(edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
					HACD_ASSERT (borderEdge);
					if (!(edgeInconflict.Find(borderEdge) || edgeInconflict.Find(borderEdge->m_twin))) {
						edgeInconflict.Insert(borderEdge, borderEdge);
					}
				}
				edge->m_mark = faceMark;
			}

			for (hacd::HaI32 i = 1; i <= perimeterCount; i ++) {
				const dgEdge* const last = perimterEdges[i - 1];
				for (dgEdge* edge = perimterEdges[i]->m_prev; edge != last; edge = edge->m_twin->m_prev) {
					if (edge->m_mark != faceMark) {
						edge->m_mark = faceMark;
						edge->m_twin->m_mark = faceMark;

						dgEdge* begin = NULL;
						for (dgEdge* ptr = edge; !begin; ptr = ptr->m_next->m_twin) {
							begin = m_mesh->FindEdge(ptr->m_next->m_incidentVertex, ptr->m_next->m_twin->m_incidentVertex);
						}
						HACD_ASSERT (begin);

						dgEdge* end = NULL;
						for (dgEdge* ptr = edge->m_twin; !end; ptr = ptr->m_next->m_twin) {
							end = m_mesh->FindEdge(ptr->m_next->m_incidentVertex, ptr->m_next->m_twin->m_incidentVertex);
						}
						HACD_ASSERT (end);
						dgEdge* const newEdge = m_mesh->ConectVertex(end, begin);
						HACD_ASSERT (!edgeInconflict.Find(newEdge));
						edgeInconflict.Insert(newEdge, newEdge);
					}
				}
			}
		}
	}

	void RecoverFaces ()
	{
		// recover all sub faces into a temporary mesh		
		bool allFaceFound = true;

//		dgIndexMapPair* const indexMap = &m_indexMap[0];
		do {
			dgMeshEffect tmpMesh (*m_mesh);
			hacd::HaI32 mark = m_mesh->IncLRU(); 

			dgTree<dgEdge*, dgEdge*> edgeInconflict(GetAllocator());
			dgMeshEffect::Iterator iter(tmpMesh);
			for (iter.Begin(); iter; iter ++) {
				dgEdge* const face = &iter.GetNode()->GetInfo();
				if (face->m_mark != mark){
					RecoverFace (tmpMesh, face, mark, -1, edgeInconflict);
				}
			}

			// if there are missing sub faces then we must recover those by insertion point on the sub edges of the missing faces
			allFaceFound = true;
			if (edgeInconflict.GetCount()) {
				HACD_ASSERT (0);
/*
				allFaceFound = false;

				dgTree<dgEdge*, dgEdge*>::Iterator iter (edgeInconflict);
				for (iter.Begin(); iter; iter ++) {
					dgEdge* const missingEdge = iter.GetNode()->GetInfo();

					hacd::HaI32 k0 = missingEdge->m_incidentVertex;
					hacd::HaI32 k1 = missingEdge->m_twin->m_incidentVertex;
					hacd::HaI32 i0 = indexMap[k0].m_convexIndex;
					hacd::HaI32 i1 = indexMap[k1].m_convexIndex;

					const dgBigVector& p0 = GetVertex(i0);
					const dgBigVector& p1 = GetVertex(i1);
					hacd::HaF32 spliteParam = hacd::HaF32 (0.5f);

					dgEdge* const newEdge = m_mesh->InsertEdgeVertex (missingEdge, spliteParam);
					dgBigVector p (p1.Add4(p0).Scale4 (spliteParam));
					hacd::HaI32 index = AddVertex(p);
					HACD_ASSERT (index != -1);
					indexMap[newEdge->m_twin->m_incidentVertex].m_convexIndex = index;
				}
				RecoverEdges ();
*/
			}
#ifdef _DEBUG
			if (allFaceFound) {
//				HACD_ASSERT (0);
/*
				dgFaceKeyMap faceMap (GetAllocator());
				for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
					dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
					for (hacd::HaI32 i = 0; i < 4; i ++) {
						dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[i];
						dgEdgeFaceKey key (face.m_index[0], face.m_index[1], face.m_index[2]);
						HACD_ASSERT (!faceMap.Find(key));
						faceMap.Insert (node, key);
					}
				}

				hacd::HaI32 mark = tmpMesh.IncLRU(); 
				for (iter.Begin(); iter; iter ++) {
					dgEdge* const face = &iter.GetNode()->GetInfo();
					if (face->m_mark != mark){
						dgEdge* ptr = face;
						do {
							ptr->m_mark = mark;
							ptr = ptr->m_next;
						} while (ptr != face);
						dgEdgeFaceKey key (face->m_incidentVertex, face->m_next->m_incidentVertex, face->m_next->m_next->m_incidentVertex);
						HACD_ASSERT (faceMap.Find(key));
					}
				}
*/
			}
#endif


		} while (!allFaceFound);


		// all faces are present in the mesh now we can recover the mesh
		// remove all tetrahedral with negative volume
		RemoveUpperHull ();

		// remove any tetrahedron that by round off error might have more that three point on on a face
//		RemoveDegeneratedTetras ();


		//hacd::HaI32 tetraMark = 1;
		hacd::HaI32 tetraMark = IncMark();
		hacd::HaI32 mark = m_mesh->IncLRU(); 

		dgTree<dgEdge*, dgEdge*> edgeInconflict(GetAllocator());
		dgMeshEffect::Iterator iter(*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const face = &iter.GetNode()->GetInfo();
			if (face->m_mark != mark){
				dgEdge* ptr = face;
				//dgTrace (("%d:", ptr->m_incidentFace))
				do {
					ptr->m_mark = mark;
					//dgTrace ((" %d", ptr->m_incidentVertex))
					ptr = ptr->m_next;
				} while (ptr != face);
				//dgTrace (("\n"));
				RecoverFace (*m_mesh, face, mark, tetraMark, edgeInconflict);
			}
		}

		// color codes all tetrahedron inside the mesh volume
		for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
			if (tetra->GetMark() == tetraMark) {
				hacd::HaI32 stack = 0;
				dgConvexHull4dTetraherum* stackPool[1024 * 4];
				for (hacd::HaI32 i = 0; i < 4; i ++) {
					dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[i];
					if (face.m_twin && (face.m_twin->GetInfo().GetMark() != tetraMark)) {
						stackPool[stack] = &face.m_twin->GetInfo();
						stack ++;
					}
				}

				while (stack) {
					stack --;
					dgConvexHull4dTetraherum* const skinTetra = stackPool[stack];
					skinTetra->SetMark (tetraMark);
					for (hacd::HaI32 i = 0; i < 4; i ++) {
						dgConvexHull4dTetraherum::dgTetrahedrumFace& face = skinTetra->m_faces[i];
						if (face.m_twin && (face.m_twin->GetInfo().GetMark() != tetraMark)) {
							stackPool[stack] = &face.m_twin->GetInfo();
							stack ++;
							HACD_ASSERT (stack < hacd::HaI32 (sizeof (stackPool) / sizeof (stackPool[0])));
						}
					}
				}
			}
		}

		// remove all tetrahedron outsize the mesh volume (those who are not painted)
		dgListNode* nextNode = NULL;
		for (dgListNode* node = GetFirst(); node; node = nextNode) {
			nextNode = node->GetNext();
			dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
			if (tetra->GetMark() != tetraMark) {
				DeleteFace (node);
			}
		}
	}


	dgMeshEffect* m_mesh;
	dgEdgeMap m_edgeMap;
	dgVertexMap m_vertexMap;
	dgMissingEdges m_missinEdges;
	dgArray<dgIndexMapPair> m_indexMap;
};


#else


/*
class Tetrahedralization: public dgDelaunayTetrahedralization
{
	class dgIndexMapPair
	{
		public:
		hacd::HaI32 m_meshIndex;
		hacd::HaI32 m_convexIndex;
	};

	class dgMissingEdges: public dgList<dgPolyhedra::dgTreeNode*> 
	{	
		public:
		dgMissingEdges ()
			:dgList<dgPolyhedra::dgTreeNode*> ()
		{
		}
		~dgMissingEdges()
		{
		}
	};

	class dgEdgeSharedTetras: public dgList<dgDelaunayTetrahedralization::dgListNode*>
	{
		public:
		dgEdgeSharedTetras(const dgEdgeSharedTetras& copy)
			:dgList<dgDelaunayTetrahedralization::dgListNode*>(copy.GetAllocator())
		{
		}

		dgEdgeSharedTetras()
			:dgList<dgDelaunayTetrahedralization::dgListNode*>()
		{
		}

		~dgEdgeSharedTetras ()
		{
		}
	};

	class dgEdgeMap: public dgTree<dgEdgeSharedTetras, hacd::HaU64>
	{
		public:
		dgEdgeMap()
			:dgTree<dgEdgeSharedTetras, hacd::HaU64>()
		{
		}

		~dgEdgeMap()
		{
			while(GetRoot()) {
				dgEdgeSharedTetras& header = GetRoot()->GetInfo();
				header.RemoveAll();
				Remove(GetRoot());
			}
		}
	};

#ifdef _DEBUG
	class dgEdgeFaceKey
	{
		public:
		dgEdgeFaceKey ()
		{}

		dgEdgeFaceKey (hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2)
		{
			m_index[0] = i0;
			m_index[1] = i1;
			m_index[2] = i2;
			while ((m_index[0] > m_index[1]) || (m_index[0] > m_index[2])) {
				i0 = m_index[0];
				m_index[0] = m_index[1];
				m_index[1] = m_index[2];
				m_index[2] = i0;
			}
		}

		hacd::HaI32 Compared (const dgEdgeFaceKey& key) const 
		{
			for (hacd::HaI32 i = 0; i < 3; i ++) {
				if (m_index[i] < key.m_index[i]) {
					return -1;
				} else if (m_index[i] > key.m_index[i]) {
					return 1;
				}
			}
			return 0;
		}


		bool operator < (const dgEdgeFaceKey& key) const 
		{
			return (Compared (key) < 0);
		}

		bool operator > (const dgEdgeFaceKey& key) const 
		{
			return (Compared (key) > 0);
		}


		hacd::HaI32 m_index[3];

	};


	class dgFaceKeyMap: public dgTree<dgListNode*, dgEdgeFaceKey>
	{
		public:
		dgFaceKeyMap()
			:dgTree<dgListNode*, dgEdgeFaceKey>()
		{

		}
	};
#endif

	public:
	Tetrahedralization (dgMeshEffect& mesh)
		:dgDelaunayTetrahedralization (mesh.GetAllocator(), mesh.GetVertexPool(), mesh.GetVertexCount(), sizeof (dgVector), 0.0f),
		m_mesh (&mesh),
		m_edgeMap (mesh.GetAllocator()), 
		m_missinEdges(mesh.GetAllocator()), 
		m_indexMap (mesh.GetVertexCount() * 8 + 2048, mesh.GetAllocator())
	{
		if (GetCount()) {
			// add every edge of each tetrahedral to a edge list
			BuildTetrahedraEdgeList ();

			// make a index map to quickly find vertex mapping form the mesh to the delaunay tetrahedron
			CreateIndexMap ();

			// Insert all missing edge in mesh as a new into the tetrahedral list
			RecoverEdges ();

			// Recover the solid mesh from the delaunay tetrahedron	
			RecoverFaces ();
		}
	}


	~Tetrahedralization()
	{
	}

	void BuildTetrahedraEdgeList () 
	{
		for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			AddEdges (node); 
		}
	}

	void RecoverEdges ()
	{
		// split every missing edge at the center and add the two half to the triangulation
		// keep doing it until all edge are present in the triangulation.

		hacd::HaI32 mark = m_mesh->IncLRU();

		// create a list all all the edge that are in the mesh but that do not appear in the delaunay tetrahedron 
		const dgIndexMapPair* const indexMap = &m_indexMap[0];
		dgPolyhedra::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark) {
				edge->m_mark = mark;
				edge->m_twin->m_mark = mark;

				hacd::HaI32 i0 = indexMap[edge->m_incidentVertex].m_convexIndex;
				hacd::HaI32 i1 = indexMap[edge->m_twin->m_incidentVertex].m_convexIndex;
				hacd::HaU64 key = GetKey (i0, i1);
				dgEdgeMap::dgTreeNode* const edgeNode = m_edgeMap.Find(key);
				if (!edgeNode) {
					m_missinEdges.Append(iter.GetNode());
				}
			}
		}

		while (m_missinEdges.GetCount()){
			dgIndexMapPair* const indexMap = &m_indexMap[0];

			dgMissingEdges::dgListNode* missingEdgeNode = m_missinEdges.GetFirst();
			dgEdge* missingEdge = &missingEdgeNode->GetInfo()->GetInfo();

			hacd::HaI32 k0 = missingEdge->m_incidentVertex;
			hacd::HaI32 k1 = missingEdge->m_twin->m_incidentVertex;
			hacd::HaI32 i0 = indexMap[k0].m_convexIndex;
			hacd::HaI32 i1 = indexMap[k1].m_convexIndex;

			m_missinEdges.Remove(missingEdgeNode);
			hacd::HaU64 key = GetKey (i0, i1);
			if (!m_edgeMap.Find(key)) {
				const dgBigVector& p0 = GetVertex(i0);
				const dgBigVector& p1 = GetVertex(i1);
				hacd::HaF64 spliteParam = hacd::HaF64 (0.5f);
				dgEdge* const newEdge = m_mesh->InsertEdgeVertex (missingEdge, hacd::HaF32 (spliteParam));
				newEdge->m_mark = mark;
				newEdge->m_next->m_mark = mark;
				newEdge->m_twin->m_mark = mark;
				newEdge->m_twin->m_prev->m_mark = mark;

				dgBigVector p (p1.Add4(p0).Scale4 (spliteParam));
				hacd::HaI32 index = AddVertex(p);
				HACD_ASSERT (index != -1);
				indexMap[newEdge->m_twin->m_incidentVertex].m_convexIndex = index;

				i0 = indexMap[newEdge->m_incidentVertex].m_convexIndex;
				i1 = indexMap[newEdge->m_twin->m_incidentVertex].m_convexIndex;
				key = GetKey (i0, i1);
				if (!m_edgeMap.Find(key)) {
					hacd::HaI32 index0 = GetVertexIndex(i0);
					hacd::HaI32 index1 = GetVertexIndex(i1);
					dgPolyhedra::dgTreeNode* const edgeNode = m_mesh->FindEdgeNode(index0, index1);
					HACD_ASSERT (edgeNode);
					m_missinEdges.Append(edgeNode);
				}

				i0 = indexMap[newEdge->m_next->m_incidentVertex].m_convexIndex;
				i1 = indexMap[newEdge->m_next->m_twin->m_incidentVertex].m_convexIndex;
				key = GetKey (i0, i1);
				if (!m_edgeMap.Find(key)) {
					hacd::HaI32 index0 = GetVertexIndex(i0);
					hacd::HaI32 index1 = GetVertexIndex(i1);
					dgPolyhedra::dgTreeNode* const edgeNode = m_mesh->FindEdgeNode(index0, index1);
					HACD_ASSERT (edgeNode);
					m_missinEdges.Append(edgeNode);
				}
			} 
		}
	}

	void RemoveDegeneratedTetras ()
	{
		hacd::HaI32 mark = m_mesh->IncLRU(); 
		dgPolyhedra::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const faceEdge = &iter.GetNode()->GetInfo();
			hacd::HaI32 count = 0; 
			dgEdge* ptr = faceEdge;
			do {
				count ++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != faceEdge);

			if (count > 3) {
				dgEdge* ptr = faceEdge;
				do {
					hacd::HaI32 k0 = m_indexMap[ptr->m_incidentVertex].m_convexIndex;
					hacd::HaI32 k1 = m_indexMap[ptr->m_next->m_incidentVertex].m_convexIndex;
					hacd::HaU64 key = GetKey (k0, k1);
					dgEdgeMap::dgTreeNode* const edgeNode = m_edgeMap.Find(key);
					if (edgeNode) {
						dgEdgeSharedTetras& header = edgeNode->GetInfo();
						for (dgEdgeSharedTetras::dgListNode* ptr1 = header.GetFirst(); ptr1; ptr1 = ptr1->GetNext()) {
							dgListNode* const tetraNode = ptr1->GetInfo();
							dgConvexHull4dTetraherum* const tetra = &tetraNode->GetInfo();
							const dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[0];
							hacd::HaI32 index[4];
							index[0] = GetVertexIndex(face.m_index[0]);
							index[1] = GetVertexIndex(face.m_index[1]);
							index[2] = GetVertexIndex(face.m_index[2]);
							index[3] = GetVertexIndex(face.m_otherVertex);

							hacd::HaI32 duplicates = 0;
							dgEdge* ptr3 = faceEdge;
							do {
								for (hacd::HaI32 i = 0; i < 4; i ++) {
									duplicates += (ptr3->m_incidentVertex == index[i]) ? 1 : 0;
								}
								ptr3 = ptr3->m_next;
							} while (ptr3 != faceEdge);
							if (duplicates > 3) {
								DeleteFace(tetraNode);
								break;
							}
						}
					}

					ptr = ptr->m_next;
				} while (ptr != faceEdge);
			}
		}
	}


	bool MatchFace (dgMeshEffect& mesh, dgEdge* const faceEdge, hacd::HaI32 tetraMark) const
	{
		hacd::HaI32 k0 = m_indexMap[faceEdge->m_incidentVertex].m_convexIndex;
		hacd::HaI32 k1 = m_indexMap[faceEdge->m_next->m_incidentVertex].m_convexIndex;
		hacd::HaU64 key = GetKey (k0, k1);
		dgEdgeMap::dgTreeNode* const edgeNode = m_edgeMap.Find(key);

		HACD_ASSERT (edgeNode);
		dgEdgeSharedTetras& header = edgeNode->GetInfo();
		for (dgEdgeSharedTetras::dgListNode* ptr = header.GetFirst(); ptr; ptr = ptr->GetNext()) {
			dgListNode* const tetraNode = ptr->GetInfo();
			dgConvexHull4dTetraherum* const tetra = &tetraNode->GetInfo();
			for (hacd::HaI32 i = 0; i < 4; i ++) {
				dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[i];
				hacd::HaI32 i0 = face.m_index[0];
				hacd::HaI32 i1 = face.m_index[1];
				hacd::HaI32 i2 = face.m_index[2];

				if (((i0 == k0) && (i1 == k1)) || ((i1 == k0) && (i2 == k1)) || ((i2 == k0) && (i0 == k1))) {
					hacd::HaI32 index[3];

					index[0] = GetVertexIndex (i0);
					index[1] = GetVertexIndex (i1);
					index[2] = GetVertexIndex (i2);
					while (index[0] != faceEdge->m_incidentVertex) {
						hacd::HaI32 tmp = index[0];
						index[0] = index[1];
						index[1] = index[2];
						index[2] = tmp;
					}
					HACD_ASSERT (index[0] == faceEdge->m_incidentVertex);
					HACD_ASSERT (index[1] == faceEdge->m_next->m_incidentVertex);

					dgEdge* nextEdge = faceEdge->m_next->m_next;
					do {
						if (nextEdge->m_incidentVertex == index[2]) {
							break;
						}
						nextEdge = nextEdge->m_next;
					} while (nextEdge != faceEdge);

					if (nextEdge != faceEdge) {
						#ifdef _MSC_VER
							#ifdef _DEBUG
								if (nextEdge->m_prev != faceEdge->m_next) {
									dgEdge* const edge = mesh.ConectVertex(faceEdge->m_next, nextEdge);
									HACD_ASSERT (edge);
									HACD_ASSERT (edge->m_next);
									HACD_ASSERT (edge->m_prev);
									HACD_ASSERT (edge->m_twin->m_next);
									HACD_ASSERT (edge->m_twin->m_prev);
									HACD_ASSERT (faceEdge->m_next == edge->m_twin);
								}
							#endif
						#endif
						if (nextEdge->m_next != faceEdge) {
							#ifdef	_DEBUG
								dgEdge* const edge = mesh.ConectVertex(faceEdge, nextEdge);
								HACD_ASSERT (edge);
								HACD_ASSERT (edge->m_next);
								HACD_ASSERT (edge->m_prev);
								HACD_ASSERT (edge->m_twin->m_next);
								HACD_ASSERT (edge->m_twin->m_prev);
								HACD_ASSERT (faceEdge->m_prev == edge);
							#else
								mesh.ConectVertex(faceEdge, nextEdge);
							#endif

						}

						if (tetraMark != -1) {
							tetra->SetMark (tetraMark);
							face.m_twin = NULL;
						}
						return true;
					}
				}
			}
		}
		return false;
	}

	void RecoverFace (dgMeshEffect& mesh, dgEdge* const face, hacd::HaI32 faceMark, hacd::HaI32 tetraMark, dgTree<dgEdge*, dgEdge*>& edgeInconflict) const
	{
		hacd::HaI32 count = 0;
		hacd::HaI32 perimeterCount = 0;
		dgEdge* edgeArray[1024];
		dgEdge* perimterEdges[1024 + 1];

		dgEdge* ptr = face;
		do {
			edgeArray[count] = ptr;
			perimterEdges[count] = ptr;
			count ++;
			HACD_ASSERT (count < hacd::HaI32 (sizeof (edgeArray) / sizeof (edgeArray[0])));
			ptr = ptr->m_next;
		} while (ptr != face);
		perimeterCount = count;
		perimterEdges[count] = face;


		while (count) {
			count --;
			dgEdge* const triangleFace = edgeArray[count];
			bool state = MatchFace (mesh, triangleFace, tetraMark);
			if (state) {
				HACD_ASSERT (triangleFace == triangleFace->m_next->m_next->m_next);
				triangleFace->m_mark = faceMark;
				dgEdge* ptr = triangleFace->m_next; 
				do {
					ptr->m_mark = faceMark;
					for (hacd::HaI32 i = 0; i < count; i ++) {
						if (ptr == edgeArray[i]) {
							edgeArray[i] = edgeArray[count - 1];
							i --;
							count --;
							break;
						}
					}
					ptr = ptr->m_next;
				} while (ptr != triangleFace);
			}
		}
		
		HACD_ASSERT (count == 0);
		for (hacd::HaI32 i = 1; i <= perimeterCount; i ++) {
			dgEdge* const last = perimterEdges[i - 1];
			for (dgEdge* edge = perimterEdges[i]->m_prev; edge != last; edge = edge->m_twin->m_prev) {
				if (edge->m_mark != faceMark) {
					hacd::HaI32 index = 0;
					for (index = 0; index < count; index ++) {
						if ((edgeArray[index] == edge) || (edgeArray[index] == edge->m_twin)) {
							break;
						}
					}
					if (index == count) {
						edgeArray[count] = edge;
						count ++;
					}
				}
			}
		}

		if (count) {
			while (count) {
				count --;
				dgEdge* const triangleFace = edgeArray[count];
				bool state = MatchFace (mesh, triangleFace, tetraMark);
				if (state) {
					HACD_ASSERT (triangleFace == triangleFace->m_next->m_next->m_next);
					triangleFace->m_mark = faceMark;
					dgEdge* ptr = triangleFace->m_next; 
					do {
						ptr->m_mark = faceMark;
						for (hacd::HaI32 i = 0; i < count; i ++) {
							if (ptr == edgeArray[i]) {
								edgeArray[i] = edgeArray[count - 1];
								i --;
								count --;
								break;
							}
						}
						ptr = ptr->m_next;
					} while (ptr != triangleFace);
				}
			}

			HACD_ASSERT (count == 0);
			for (hacd::HaI32 i = 0; i < perimeterCount; i ++) {
				dgEdge* const edge = perimterEdges[i];
				if (edge->m_mark != faceMark) {
					dgEdge* const borderEdge = m_mesh->FindEdge(edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
					HACD_ASSERT (borderEdge);
					if (!(edgeInconflict.Find(borderEdge) || edgeInconflict.Find(borderEdge->m_twin))) {
						edgeInconflict.Insert(borderEdge, borderEdge);
					}
				}
				edge->m_mark = faceMark;
			}
			
			for (hacd::HaI32 i = 1; i <= perimeterCount; i ++) {
				const dgEdge* const last = perimterEdges[i - 1];
				for (dgEdge* edge = perimterEdges[i]->m_prev; edge != last; edge = edge->m_twin->m_prev) {
					if (edge->m_mark != faceMark) {
						edge->m_mark = faceMark;
						edge->m_twin->m_mark = faceMark;

						dgEdge* begin = NULL;
						for (dgEdge* ptr = edge; !begin; ptr = ptr->m_next->m_twin) {
							begin = m_mesh->FindEdge(ptr->m_next->m_incidentVertex, ptr->m_next->m_twin->m_incidentVertex);
						}
						HACD_ASSERT (begin);

						dgEdge* end = NULL;
						for (dgEdge* ptr = edge->m_twin; !end; ptr = ptr->m_next->m_twin) {
							end = m_mesh->FindEdge(ptr->m_next->m_incidentVertex, ptr->m_next->m_twin->m_incidentVertex);
						}
						HACD_ASSERT (end);
						dgEdge* const newEdge = m_mesh->ConectVertex(end, begin);
						HACD_ASSERT (!edgeInconflict.Find(newEdge));
						edgeInconflict.Insert(newEdge, newEdge);
					}
				}
			}
		}
	}


	static hacd::HaI32 ConvexCompareIndex(const dgIndexMapPair* const  A, const dgIndexMapPair* const B, void* const context)
	{
		if (A->m_meshIndex > B->m_meshIndex) {
			return 1;
		} else if (A->m_meshIndex < B->m_meshIndex) {
			return -1;
		}
		return 0;
	}

	void CreateIndexMap ()
	{
		// make a index map to quickly find vertex mapping form the mesh to the delaunay tetrahedron
		m_indexMap[GetVertexCount()].m_meshIndex = 0;
		dgIndexMapPair* const indexMap = &m_indexMap[0];
		for (hacd::HaI32 i = 0; i < GetVertexCount(); i ++) {
			indexMap[i].m_convexIndex = i;
			indexMap[i].m_meshIndex = GetVertexIndex(i);
		}
		dgSort(indexMap, GetVertexCount(), ConvexCompareIndex);
	}

	hacd::HaU64 GetKey (hacd::HaI32 i0, hacd::HaI32 i1) const
	{
		return (i1 > i0) ?  (hacd::HaU64 (i1) << 32) + i0 : (hacd::HaU64 (i0) << 32) + i1;
	}

	void InsertNewNode (hacd::HaI32 i0, hacd::HaI32 i1, dgListNode* const node)
	{
		hacd::HaU64 key = GetKey (i1, i0);

		dgEdgeMap::dgTreeNode* edgeNode = m_edgeMap.Find(key);
		if (!edgeNode) {
			dgEdgeSharedTetras tmp (GetAllocator());
			edgeNode = m_edgeMap.Insert(tmp, key);
		}
		dgEdgeSharedTetras& header = edgeNode->GetInfo();

		#ifdef _DEBUG
		for (dgEdgeSharedTetras::dgListNode* ptr = header.GetFirst(); ptr; ptr = ptr->GetNext()) {
			HACD_ASSERT (ptr->GetInfo() != node);
		}
		#endif
		header.Append(node);
	}

	void AddEdges (dgListNode* const node) 
	{
		dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
		hacd::HaF64 volume = GetTetraVolume (tetra);
		if (volume < hacd::HaF64 (0.0f)) {
			const dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[0];
			for (hacd::HaI32 i = 0; i < 3; i ++) {
				hacd::HaI32 i0 = face.m_otherVertex;
				hacd::HaI32 i1 = face.m_index[i];
				InsertNewNode (i0, i1, node);
			}

			hacd::HaI32 i0 = face.m_index[2];
			for (hacd::HaI32 i = 0; i < 3; i ++) {
				hacd::HaI32 i1 = face.m_index[i];
				InsertNewNode (i0, i1, node);
				i0 = i1;
			}
		}
	}

	void RemoveNode(hacd::HaI32 i0, hacd::HaI32 i1, dgListNode* const node)
	{
		hacd::HaU64 key = GetKey (i0, i1);

		dgEdgeMap::dgTreeNode* const edgeNode = m_edgeMap.Find(key);
		if (edgeNode) {
			dgEdgeSharedTetras& header = edgeNode->GetInfo();
			for (dgEdgeSharedTetras::dgListNode* ptr = header.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dgListNode* const me = ptr->GetInfo();
				if (me == node)  {
					header.Remove(ptr);
					if (!header.GetCount()) {
						m_edgeMap.Remove(edgeNode);
					}

					hacd::HaI32 index0 = GetVertexIndex(i0);
					hacd::HaI32 index1 = GetVertexIndex(i1);
					dgPolyhedra::dgTreeNode* const edgeNode = m_mesh->FindEdgeNode(index0, index1);
					if(edgeNode) {
						m_missinEdges.Append(edgeNode);
					}
					break;
				}
			}
		}
	}

	void RemoveEdges (dgListNode* const node)
	{
		dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
		const dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[0];
		for (hacd::HaI32 i = 0; i < 3; i ++) {
			hacd::HaI32 i0 = face.m_otherVertex;
			hacd::HaI32 i1 = face.m_index[i];
			RemoveNode(i0, i1, node);
		}

		hacd::HaI32 i0 = face.m_index[2];
		for (hacd::HaI32 i = 0; i < 3; i ++) {
			hacd::HaI32 i1 = face.m_index[i];
			RemoveNode(i0, i1, node);
			i0 = i1;
		}
	}

	dgListNode* AddFace (hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2, hacd::HaI32 i3)
	{
		dgListNode* const face = dgDelaunayTetrahedralization::AddFace(i0, i1, i2, i3);
		AddEdges(face);
		return face;
	}

	void DeleteFace (dgListNode* const node) 
	{
		RemoveEdges (node);

		dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
		for (hacd::HaI32 i= 0; i < 4; i ++) {
			const dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[i];
			dgListNode* const twinNode = face.m_twin;
			if (twinNode) {
				dgConvexHull4dTetraherum* const twinTetra = &twinNode->GetInfo();
				for (hacd::HaI32 i = 0; i < 4; i ++) {
					if (twinTetra->m_faces[i].m_twin == node) {
						twinTetra->m_faces[i].m_twin = NULL;
					}
				}
			}
		}
		
		dgDelaunayTetrahedralization::DeleteFace(node);
	}




	void RecoverFaces ()
	{
		// recover all sub faces into a temporary mesh		
		bool allFaceFound = true;

		dgIndexMapPair* const indexMap = &m_indexMap[0];
		do {
			dgMeshEffect tmpMesh (*m_mesh);
			hacd::HaI32 mark = m_mesh->IncLRU(); 

			dgTree<dgEdge*, dgEdge*> edgeInconflict(GetAllocator());
			dgMeshEffect::Iterator iter(tmpMesh);
			for (iter.Begin(); iter; iter ++) {
				dgEdge* const face = &iter.GetNode()->GetInfo();
				if (face->m_mark != mark){
					RecoverFace (tmpMesh, face, mark, -1, edgeInconflict);
				}
			}

			// if there are missing sub faces then we must recover those by insertion point on the sub edges of the missing faces
			allFaceFound = true;
			if (edgeInconflict.GetCount()) {
				allFaceFound = false;

				dgTree<dgEdge*, dgEdge*>::Iterator iter (edgeInconflict);
				for (iter.Begin(); iter; iter ++) {
					dgEdge* const missingEdge = iter.GetNode()->GetInfo();

					hacd::HaI32 k0 = missingEdge->m_incidentVertex;
					hacd::HaI32 k1 = missingEdge->m_twin->m_incidentVertex;
					hacd::HaI32 i0 = indexMap[k0].m_convexIndex;
					hacd::HaI32 i1 = indexMap[k1].m_convexIndex;

					const dgBigVector& p0 = GetVertex(i0);
					const dgBigVector& p1 = GetVertex(i1);
					hacd::HaF32 spliteParam = hacd::HaF32 (0.5f);

					dgEdge* const newEdge = m_mesh->InsertEdgeVertex (missingEdge, spliteParam);
					dgBigVector p (p1.Add4(p0).Scale4 (spliteParam));
					hacd::HaI32 index = AddVertex(p);
					HACD_ASSERT (index != -1);
					indexMap[newEdge->m_twin->m_incidentVertex].m_convexIndex = index;
				}
				RecoverEdges ();
			}

			#ifdef _DEBUG
			if (allFaceFound) {
				dgFaceKeyMap faceMap (GetAllocator());
				for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
					dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
					for (hacd::HaI32 i = 0; i < 4; i ++) {
						dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[i];
						dgEdgeFaceKey key (face.m_index[0], face.m_index[1], face.m_index[2]);
						HACD_ASSERT (!faceMap.Find(key));
						faceMap.Insert (node, key);
					}
				}

				hacd::HaI32 mark = tmpMesh.IncLRU(); 
				for (iter.Begin(); iter; iter ++) {
					dgEdge* const face = &iter.GetNode()->GetInfo();
					if (face->m_mark != mark){
						dgEdge* ptr = face;
						do {
							ptr->m_mark = mark;
							ptr = ptr->m_next;
						} while (ptr != face);
						dgEdgeFaceKey key (face->m_incidentVertex, face->m_next->m_incidentVertex, face->m_next->m_next->m_incidentVertex);
						HACD_ASSERT (faceMap.Find(key));
					}
				}
			}
			#endif


		} while (!allFaceFound);


		// all faces are present in the mesh now we can recover the mesh
		// remove all tetrahedral with negative volume
		RemoveUpperHull ();

		// remove any tetrahedron that by round off error might have more that three point on on a face
		RemoveDegeneratedTetras ();


		//hacd::HaI32 tetraMark = 1;
		hacd::HaI32 tetraMark = IncMark();
		hacd::HaI32 mark = m_mesh->IncLRU(); 

		dgTree<dgEdge*, dgEdge*> edgeInconflict(GetAllocator());
		dgMeshEffect::Iterator iter(*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const face = &iter.GetNode()->GetInfo();
			if (face->m_mark != mark){
				dgEdge* ptr = face;
				//dgTrace (("%d:", ptr->m_incidentFace))
				do {
					ptr->m_mark = mark;
					//dgTrace ((" %d", ptr->m_incidentVertex))
					ptr = ptr->m_next;
				} while (ptr != face);
				//dgTrace (("\n"));
				RecoverFace (*m_mesh, face, mark, tetraMark, edgeInconflict);
			}
		}

		// color codes all tetrahedron inside the mesh volume
		for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
			if (tetra->GetMark() == tetraMark) {
				hacd::HaI32 stack = 0;
				dgConvexHull4dTetraherum* stackPool[1024 * 4];
				for (hacd::HaI32 i = 0; i < 4; i ++) {
					dgConvexHull4dTetraherum::dgTetrahedrumFace& face = tetra->m_faces[i];
					if (face.m_twin && (face.m_twin->GetInfo().GetMark() != tetraMark)) {
						stackPool[stack] = &face.m_twin->GetInfo();
						stack ++;
					}
				}

				while (stack) {
					stack --;
					dgConvexHull4dTetraherum* const skinTetra = stackPool[stack];
					skinTetra->SetMark (tetraMark);
					for (hacd::HaI32 i = 0; i < 4; i ++) {
						dgConvexHull4dTetraherum::dgTetrahedrumFace& face = skinTetra->m_faces[i];
						if (face.m_twin && (face.m_twin->GetInfo().GetMark() != tetraMark)) {
							stackPool[stack] = &face.m_twin->GetInfo();
							stack ++;
							HACD_ASSERT (stack < hacd::HaI32 (sizeof (stackPool) / sizeof (stackPool[0])));
						}
					}
				}
			}
		}

		// remove all tetrahedron outsize the mesh volume (those who are not painted)
		dgListNode* nextNode = NULL;
		for (dgListNode* node = GetFirst(); node; node = nextNode) {
			nextNode = node->GetNext();
			dgConvexHull4dTetraherum* const tetra = &node->GetInfo();
			if (tetra->GetMark() != tetraMark) {
				DeleteFace (node);
			}
		}
	}


	dgMeshEffect* m_mesh;
	dgEdgeMap m_edgeMap;
	dgMissingEdges m_missinEdges;
	dgArray<dgIndexMapPair> m_indexMap;
};
*/


#endif

dgMeshEffect* dgMeshEffect::CreateDelanayTretrahedralization (hacd::HaI32 interionMaterial, dgMatrix& matrix) const
{
	HACD_ASSERT (0);
	return NULL;
}


dgMeshEffect* dgMeshEffect::CreateVoronoiPartition (hacd::HaI32 pointsCount, hacd::HaI32 pointStrideInBytes, const hacd::HaF32* const pointCloud, hacd::HaI32 interiorMaterial, dgMatrix& textureProjectionMatrix) const
{
	HACD_ALWAYS_ASSERT(); // not implemented in the reduced source version
#if 0  // UNUSED

//return new (GetAllocator()) dgMeshEffect (*this);

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	hacd::HaU32 controlWorld = dgControlFP (0xffffffff, 0);
	dgControlFP (_PC_53, _MCW_PC);
#endif

	dgMeshEffectSolidTree* const tree = CreateSolidTree();
	HACD_ASSERT (tree);

	dgStack<dgBigVector> buffer(pointsCount);
	dgBigVector* const pool = &buffer[0];

	hacd::HaI32 count = 0;
	hacd::HaF64 quantizeFactor = hacd::HaF64 (16.0f);
	hacd::HaF64 invQuantizeFactor = hacd::HaF64 (1.0f) / quantizeFactor;
	hacd::HaI32 stride = pointStrideInBytes / sizeof (hacd::HaF32); 
	for (hacd::HaI32 i = 0; i < pointsCount; i ++) {
		hacd::HaF64 x = pointCloud[i * stride + 0];
		hacd::HaF64 y	= pointCloud[i * stride + 1];
		hacd::HaF64 z	= pointCloud[i * stride + 2];
		x = floor (x * quantizeFactor) * invQuantizeFactor;
		y = floor (y * quantizeFactor) * invQuantizeFactor;
		z = floor (z * quantizeFactor) * invQuantizeFactor;
		dgBigVector p (x, y, z, hacd::HaF64 (0.0f));

		if (tree->GetPointSide (p) == dgMeshEffectSolidTree::m_solid) {
			pool[count] = p;
			count ++;
		}
	}

	HACD_ASSERT (count >= 4);
	dgStack<hacd::HaI32> indexList(count);
	count = dgVertexListToIndexList(&pool[0].m_x, sizeof (dgBigVector), 3, count, &indexList[0], hacd::HaF64 (1.0e-5f));	
	HACD_ASSERT (count >= 4);

	dgDelaunayTetrahedralization delaunayTetrahedras (&pool[0].m_x, count, sizeof (dgBigVector), 0.0f);
	delaunayTetrahedras.RemoveUpperHull ();

	dgBigVector minBox;
	dgBigVector maxBox;
	CalculateAABB (minBox, maxBox);
	maxBox -= minBox;
	hacd::HaF32 bboxDiagnalFactor = 4.0f;
	hacd::HaF64 perimeterConvexBound = bboxDiagnalFactor * sqrt(maxBox % maxBox);

	hacd::HaI32 tetraCount = delaunayTetrahedras.GetCount();
	dgStack<dgBigVector> voronoiPoints(tetraCount);
	dgStack<dgDelaunayTetrahedralization::dgListNode*> tetradrumNode(tetraCount);
	dgTree<dgList<hacd::HaI32>, hacd::HaI32> delanayNodes ();	

	hacd::HaI32 index = 0;
	const dgHullVector* const delanayPoints = delaunayTetrahedras.GetHullVertexArray();
	for (dgDelaunayTetrahedralization::dgListNode* node = delaunayTetrahedras.GetFirst(); node; node = node->GetNext()) {
		dgConvexHull4dTetraherum& tetra = node->GetInfo();
		dgBigVector origin (tetra.CircumSphereCenter (delanayPoints));
		voronoiPoints[index] = dgBigVector (hacd::HaF64 (origin.m_x), hacd::HaF64 (origin.m_y), hacd::HaF64 (origin.m_z), hacd::HaF64 (0.0f));
		tetradrumNode[index] = node;

		for (hacd::HaI32 i = 0; i < 3; i ++) {
			dgTree<dgList<hacd::HaI32>, hacd::HaI32>::dgTreeNode* header = delanayNodes.Find(tetra.m_faces[0].m_index[i]);
			if (!header) {
				dgList<hacd::HaI32> list ();
				header = delanayNodes.Insert(list, tetra.m_faces[0].m_index[i]);
			}
			header->GetInfo().Append (index);
		}

		dgTree<dgList<hacd::HaI32>, hacd::HaI32>::dgTreeNode* header = delanayNodes.Find(tetra.m_faces[0].m_otherVertex);
		if (!header) {
			dgList<hacd::HaI32> list ();
			header = delanayNodes.Insert(list, tetra.m_faces[0].m_otherVertex);
		}
		header->GetInfo().Append (index);
		index ++;
	}


	dgMeshEffect* const voronoiPartition = HACD_NEW(dgMeshEffect)(true);
	voronoiPartition->BeginPolygon();
	hacd::HaF64 layer = hacd::HaF64 (0.0f);

	dgTree<dgList<hacd::HaI32>, hacd::HaI32>::Iterator iter (delanayNodes);
	for (iter.Begin(); iter; iter ++) {

		hacd::HaI32 count = 0;
		dgBigVector pointArray[256];
		dgTree<dgList<hacd::HaI32>, hacd::HaI32>::dgTreeNode* const nodeNode = iter.GetNode();

		dgList<hacd::HaI32>& list = nodeNode->GetInfo();

		hacd::HaI32 key = nodeNode->GetKey();

		for (dgList<hacd::HaI32>::dgListNode* ptr = list.GetFirst(); ptr; ptr = ptr->GetNext()) {
			hacd::HaI32 i = ptr->GetInfo();
			dgConvexHull4dTetraherum* const tetrahedrum = &tetradrumNode[i]->GetInfo();
			for (hacd::HaI32 j = 0; j < 4; j ++) {
				if (!tetrahedrum->m_faces[j].m_twin) {
					if ((tetrahedrum->m_faces[j].m_index[0] == key) || (tetrahedrum->m_faces[j].m_index[1] == key) || (tetrahedrum->m_faces[j].m_index[2] == key)) {
						dgBigVector p0 (delaunayTetrahedras.GetVertex(tetrahedrum->m_faces[j].m_index[0]));
						dgBigVector p1 (delaunayTetrahedras.GetVertex(tetrahedrum->m_faces[j].m_index[1]));
						dgBigVector p2 (delaunayTetrahedras.GetVertex(tetrahedrum->m_faces[j].m_index[2]));
						dgBigVector n ((p1 - p0) * (p2 - p0));
						n = n.Scale (hacd::HaF64 (1.0f) / sqrt(n % n));
						dgBigVector normal (hacd::HaF64 (n.m_x), hacd::HaF64 (n.m_y), hacd::HaF64  (n.m_z), hacd::HaF64 (0.0f));
						pointArray[count] = voronoiPoints[i] + normal.Scale (perimeterConvexBound);

						count ++;
						HACD_ASSERT (count < hacd::HaI32 (sizeof (pointArray) / sizeof (pointArray[0])));
					}
				}
			}

			pointArray[count] = voronoiPoints[i];
			count ++;
			HACD_ASSERT (count < hacd::HaI32 (sizeof (pointArray) / sizeof (pointArray[0])));
		}

		dgMeshEffect* const convexMesh = MakeDelanayIntersection (tree, &pointArray[0], count, interiorMaterial, textureProjectionMatrix, hacd::HaF64 (45.0f * 3.1416f / 180.0f));
		if (convexMesh) {
			for (hacd::HaI32 i = 0; i < convexMesh->m_pointCount; i ++) {
				convexMesh->m_points[i].m_w = layer;
			}
			for (hacd::HaI32 i = 0; i < convexMesh->m_atribCount; i ++) {
				convexMesh->m_attib[i].m_vertex.m_w = layer;
			}

			voronoiPartition->MergeFaces(convexMesh);
			layer += hacd::HaF64 (1.0f);

			convexMesh->Release();
		}
	}

	voronoiPartition->EndPolygon(hacd::HaF64 (1.0e-5f));

	voronoiPartition->ConvertToPolygons();

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	dgControlFP (controlWorld, _MCW_PC);
#endif

	delete tree;
	return voronoiPartition;
#else
	return NULL;
#endif
}



dgMeshEffect* dgMeshEffect::MakeDelanayIntersection (dgMeshEffectSolidTree* const tree, dgBigVector* const points, hacd::HaI32 count, hacd::HaI32 materialId, const dgMatrix& textureProjectionMatrix, hacd::HaF32 normalAngleInRadians) const
{
	for (hacd::HaI32 i = 0; i < count; i ++) {
		points[i].m_x = QuantizeCordinade(points[i].m_x);
		points[i].m_y = QuantizeCordinade(points[i].m_y);
		points[i].m_z = QuantizeCordinade(points[i].m_z);
		points[i].m_w = hacd::HaF64 (0.0f);
	}

	dgMeshEffect* intersection = NULL;
	dgMeshEffect convexMesh (&points[0].m_x, count, sizeof (dgBigVector), hacd::HaF64 (0.0f));

	if (convexMesh.GetCount()) {
		convexMesh.CalculateNormals(normalAngleInRadians);
		convexMesh.UniformBoxMapping (materialId, textureProjectionMatrix);

#if 0
intersection =  new (GetAllocator()) dgMeshEffect (convexMesh);
#else

		DG_MESG_EFFECT_BOOLEAN_INIT();

		ClipMesh (&convexMesh, &leftMeshSource, &rightMeshSource, &sourceCoplanar);
		convexMesh.ClipMesh (tree, &leftMeshClipper, &rightMeshClipper, &clipperCoplanar);
		if (leftMeshSource || leftMeshClipper) {
			result = HACD_NEW(dgMeshEffect)(true);
			result->BeginPolygon();

			if (leftMeshSource) {
				result->MergeFaces(leftMeshSource);
			}

			if (leftMeshClipper) {
				result->MergeFaces(leftMeshClipper);
			}

			if (clipperCoplanar && sourceCoplanar) {
				sourceCoplanar->FilterCoplanarFaces (clipperCoplanar, hacd::HaF32 (-1.0f));
				result->MergeFaces(sourceCoplanar);
			}

			result->EndPolygon(hacd::HaF64 (1.0e-5f));
			if (!result->GetCount()) {
				result->Release();
				result = NULL;
			}
		}
		intersection = result;
		DG_MESG_EFFECT_BOOLEAN_FINISH()
#endif
	}


#if 0
	if (intersection) {
		dgBigVector xxx (0, 0, 0, 0);
		for (hacd::HaI32 i = 0; i < intersection->m_pointCount; i ++) {
			xxx += intersection->m_points[i];
		}
		xxx = xxx.Scale (0.5f / intersection->m_pointCount);
		for (hacd::HaI32 i = 0; i < intersection->m_pointCount; i ++) {
			intersection->m_points[i] += xxx;
		}
		for (hacd::HaI32 i = 0; i < intersection->m_atribCount; i ++) {
			intersection->m_attib[i].m_vertex += xxx;
		}
	}
#endif

	return intersection;
}




