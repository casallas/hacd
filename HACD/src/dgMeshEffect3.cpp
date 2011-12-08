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

// based of the paper Hierarchical Approximate Convex Decomposition by Khaled Mamou 
// with his permission to adapt his algorithm so be more efficient.
// available http://sourceforge.net/projects/hacd/
// for the details http://kmamou.blogspot.com/


class dgClusterFace
{
	public:
	dgClusterFace()
	{
	}
	~dgClusterFace()
	{
	}

	dgEdge* m_edge;
	hacd::HaF64 m_area;
	hacd::HaF64 m_perimeter;
	dgBigVector m_normal;
};

class dgPairProxi
{
	public:
	dgPairProxi()
		:m_edgeA(NULL)
		,m_edgeB(NULL)
		,m_area(hacd::HaF64(0.0f))
		,m_perimeter(hacd::HaF64(0.0f))
	{
	}

	~dgPairProxi()
	{
	}

	dgEdge* m_edgeA;
	dgEdge* m_edgeB;
	hacd::HaF64 m_area;
	hacd::HaF64 m_perimeter;
};

class dgClusterList: public dgList<dgClusterFace>
{
	public:
	dgClusterList() 
		: dgList<dgClusterFace>()
		,m_area (hacd::HaF32 (0.0f))
		,m_perimeter (hacd::HaF32 (0.0f))
	  {
	  }

	  ~dgClusterList()
	  {
	  }

	  hacd::HaI32 AddVertexToPool(const dgMeshEffect& mesh, dgBigVector* const vertexPool, hacd::HaI32* const vertexMarks, hacd::HaI32 vertexMark)
	  {
		  hacd::HaI32 count = 0;

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& face = node->GetInfo();

			  dgEdge* edge = face.m_edge;
			  do {
				  hacd::HaI32 index = edge->m_incidentVertex;
				  if (vertexMarks[index] != vertexMark)
				  {
					  vertexMarks[index] = vertexMark;
					  vertexPool[count] = points[index];
					  count++;
				  }
				  edge = edge->m_next;
			  } while (edge != face.m_edge);
		  }
		  return count;
	  }

	  hacd::HaF64 CalculateTriangleConcavity2(const dgConvexHull3d& convexHull, dgClusterFace& info, hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2, const dgBigVector* const points) const
	  {
		  hacd::HaU32 head = 1;
		  hacd::HaU32 tail = 0;
		  dgBigVector pool[1<<8][3];

		  pool[0][0] = points[i0];
		  pool[0][1] = points[i1];
		  pool[0][2] = points[i2];

		  const dgBigVector step(info.m_normal.Scale(hacd::HaF64(4.0f) * convexHull.GetDiagonal()));

		  hacd::HaF64 concavity = hacd::HaF32(0.0f);
		  hacd::HaF64 minArea = hacd::HaF32(0.125f);
		  hacd::HaF64 minArea2 = minArea * minArea * 0.5f;

		  // weight the area by the area of the face 
		  //dgBigVector edge10(pool[0][1] - pool[0][0]);
		  //dgBigVector edge20(pool[0][2] - pool[0][0]);
		  //dgBigVector triangleArea = edge10 * edge20;
		  //hacd::HaF64 triangleArea2 = triangleArea % triangleArea;
		  //if ((triangleArea2 / minArea2)> hacd::HaF32 (64.0f)) {
			// minArea2 = triangleArea2 / hacd::HaF32 (64.0f);
		  //}

		  hacd::HaI32 maxCount = 4;
		  hacd::HaU32 mask = (sizeof (pool) / (3 * sizeof (pool[0][0]))) - 1;
		  while ((tail != head) && (maxCount >= 0)) {
			  //stack--;
			  maxCount --;
			  dgBigVector p0(pool[tail][0]);
			  dgBigVector p1(pool[tail][1]);
			  dgBigVector p2(pool[tail][2]);
			  tail =  (tail + 1) & mask;

			  dgBigVector q1((p0 + p1 + p2).Scale(hacd::HaF64(1.0f / 3.0f)));
			  dgBigVector q0(q1 + step);

			  hacd::HaF64 param = convexHull.RayCast(q0, q1);
			  if (param > hacd::HaF64(1.0f)) {
				  param = hacd::HaF64(1.0f);
			  }
			  dgBigVector dq(step.Scale(hacd::HaF32(1.0f) - param));
			  hacd::HaF64 lenght2 = dq % dq;
			  if (lenght2 > concavity) {
				  concavity = lenght2;
			  }

			  if (((head + 1) & mask) != tail) {
				  dgBigVector edge10(p1 - p0);
				  dgBigVector edge20(p2 - p0);
				  dgBigVector n(edge10 * edge20);
				  hacd::HaF64 area2 = n % n;
				  if (area2 > minArea2) {
					  dgBigVector p01((p0 + p1).Scale(hacd::HaF64(0.5f)));
					  dgBigVector p12((p1 + p2).Scale(hacd::HaF64(0.5f)));
					  dgBigVector p20((p2 + p0).Scale(hacd::HaF64(0.5f)));

					  pool[head][0] = p0;
					  pool[head][1] = p01;
					  pool[head][2] = p20;
					  head = (head + 1) & mask;

					  if (((head + 1) & mask) != tail) {
						  pool[head][0] = p1;
						  pool[head][1] = p12;
						  pool[head][2] = p01;
						  head = (head + 1) & mask;

						  if (((head + 1) & mask) != tail)	{
							  pool[head][0] = p2;
							  pool[head][1] = p20;
							  pool[head][2] = p12;
							  head = (head + 1) & mask;
						  }
					  }
				  }
			  }
		  }
		  return concavity;
	  }

	  hacd::HaF64 CalculateConcavity2(const dgConvexHull3d& convexHull, const dgMeshEffect& mesh)
	  {
		  hacd::HaF64 concavity = hacd::HaF32(0.0f);

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();

		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& info = node->GetInfo();
			  hacd::HaI32 i0 = info.m_edge->m_incidentVertex;
			  hacd::HaI32 i1 = info.m_edge->m_next->m_incidentVertex;
			  for (dgEdge* edge = info.m_edge->m_next->m_next; edge != info.m_edge; edge = edge->m_next) {
				  hacd::HaI32 i2 = edge->m_incidentVertex;
				  hacd::HaF64 val = CalculateTriangleConcavity2(convexHull, info, i0, i1, i2, points);
				  if (val > concavity) {
					  concavity = val;
				  }
				  i1 = i2;
			  }
		  }

		  return concavity;
	  }

	  bool IsClusterCoplanar(const dgBigPlane& plane,
		  const dgMeshEffect& mesh) const
	  {
		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& info = node->GetInfo();

			  dgEdge* ptr = info.m_edge;
			  do {
				  const dgBigVector& p = points[ptr->m_incidentVertex];
				  hacd::HaF64 dist = fabs(plane.Evalue(p));
				  if (dist > hacd::HaF64(1.0e-5f)) {
					  return false;
				  }
				  ptr = ptr->m_next;
			  } while (ptr != info.m_edge);
		  }

		  return true;
	  }

	  bool IsEdgeConvex(const dgBigPlane& plane, const dgMeshEffect& mesh,
		  dgEdge* const edge) const
	  {
		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  dgEdge* const edge0 = edge->m_next;
		  dgEdge* ptr = edge0->m_twin->m_next;
		  do {
			  if (ptr->m_twin->m_incidentFace == edge->m_twin->m_incidentFace) {
				  HACD_ASSERT(edge0->m_incidentVertex == ptr->m_incidentVertex);
				  dgBigVector e0(points[edge0->m_twin->m_incidentVertex] - points[edge0->m_incidentVertex]);
				  dgBigVector e1(points[ptr->m_twin->m_incidentVertex] - points[edge0->m_incidentVertex]);
				  dgBigVector normal(e0 * e1);
				  return (normal % plane) > hacd::HaF64(0.0f);
			  }
			  ptr = ptr->m_twin->m_next;
		  } while (ptr != edge->m_twin);

		  HACD_ASSERT(0);
		  return true;
	  }

	  // calculate the convex hull of a conched group of faces,
	  // and measure the concavity, according to Khaled convexity criteria, which is basically
	  // has two components, 
	  // the first is ratio  between the the perimeter of the group of faces
	  // and the second the largest distance from any of the face to the surface of the hull 
	  // when the faces are are a strip of a convex hull the perimeter ratio components is 1.0 and the distance to the hull is zero
	  // this is the ideal concavity.
	  // when the face are no part of the hull, then the worse distance to the hull is dominate the the metric
	  // this matrix is used to place all possible combination of this cluster with any adjacent cluster into a priority heap and determine 
	  // which pair of two adjacent cluster is the best selection for combining the into a larger cluster.
	  void CalculateNodeCost(dgMeshEffect& mesh, hacd::HaI32 meshMask,
		  dgBigVector* const vertexPool, hacd::HaI32* const vertexMarks,
		  hacd::HaI32& vertexMark, dgClusterList* const clusters, hacd::HaF64 diagonalInv,
		  hacd::HaF64 aspectRatioCoeficent, dgList<dgPairProxi>& proxyList,
		  dgUpHeap<dgList<dgPairProxi>::dgListNode*, hacd::HaF64>& heap)
	  {
		  hacd::HaI32 faceIndex = GetFirst()->GetInfo().m_edge->m_incidentFace;

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();

		  bool flatStrip = true;
		  dgBigPlane plane(GetFirst()->GetInfo().m_normal, -(points[GetFirst()->GetInfo().m_edge->m_incidentVertex] % GetFirst()->GetInfo().m_normal));
		  if (GetCount() > 1) {
			  flatStrip = IsClusterCoplanar(plane, mesh);
		  }

		  vertexMark++;
		  hacd::HaI32 vertexCount = AddVertexToPool(mesh, vertexPool, vertexMarks, vertexMark);
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  //dgClusterFace& clusterFaceA = GetFirst()->GetInfo();
			  dgClusterFace& clusterFaceA = node->GetInfo();

			  dgEdge* edge = clusterFaceA.m_edge;
			  do {
				  hacd::HaI32 twinFaceIndex = edge->m_twin->m_incidentFace;
				  if ((edge->m_mark != meshMask) && (twinFaceIndex != faceIndex) && (twinFaceIndex > 0)) {

					  dgClusterList& clusterListB = clusters[twinFaceIndex];

					  vertexMark++;
					  hacd::HaI32 extraCount = clusterListB.AddVertexToPool(mesh, &vertexPool[vertexCount], &vertexMarks[0], vertexMark);

					  hacd::HaI32 count = vertexCount + extraCount;
					  dgConvexHull3d convexHull(&vertexPool[0].m_x, sizeof(dgBigVector), count, 0.0);

					  hacd::HaF64 concavity = hacd::HaF64(0.0f);
					  if (convexHull.GetVertexCount()) {
						  concavity = sqrt(GetMax(CalculateConcavity2(convexHull, mesh), clusterListB.CalculateConcavity2(convexHull, mesh)));
						  if (concavity < hacd::HaF64(1.0e-3f)) {
							  concavity = hacd::HaF64(0.0f);
						  }
					  }

					  if ((concavity == hacd::HaF64(0.0f)) && flatStrip)  {
						  if (clusterListB.IsClusterCoplanar(plane, mesh)) {
							  bool concaveEdge = !(IsEdgeConvex(plane, mesh, edge) && IsEdgeConvex(plane, mesh, edge->m_twin));
							  if (concaveEdge) {
								  concavity += 1000.0f;
							  }
						  }
					  }

					  dgBigVector p1p0(points[edge->m_twin->m_incidentVertex] - points[edge->m_incidentVertex]);
					  hacd::HaF64 edgeLength = hacd::HaF64(2.0f) * sqrt(p1p0 % p1p0);

					  hacd::HaF64 area = m_area + clusterListB.m_area;
					  hacd::HaF64 perimeter = m_perimeter + clusterListB.m_perimeter - edgeLength;
					  hacd::HaF64 edgeCost = perimeter * perimeter / (hacd::HaF64(4.0f * 3.141592f) * area);
					  hacd::HaF64 cost = diagonalInv * (concavity + edgeCost * aspectRatioCoeficent);

					  if ((heap.GetCount() + 20) > heap.GetMaxCount()) {
						  for (hacd::HaI32 i = heap.GetCount() - 1; i >= 0; i--) {
							  dgList<dgPairProxi>::dgListNode* emptyNode = heap[i];
							  dgPairProxi& emptyPair = emptyNode->GetInfo();
							  if ((emptyPair.m_edgeA == NULL) && (emptyPair.m_edgeB == NULL)) {
								  heap.Remove(i);
							  }
						  }
					  }

					  dgList<dgPairProxi>::dgListNode* pairNode = proxyList.Append();
					  dgPairProxi& pair = pairNode->GetInfo();
					  pair.m_edgeA = edge;
					  pair.m_edgeB = edge->m_twin;
					  pair.m_area = area;
					  pair.m_perimeter = perimeter;
					  edge->m_userData = hacd::HaU64(pairNode);
					  edge->m_twin->m_userData = hacd::HaU64(pairNode);
					  heap.Push(pairNode, cost);
				  }

				  edge->m_mark = meshMask;
				  edge->m_twin->m_mark = meshMask;
				  edge = edge->m_next;
			  } while (edge != clusterFaceA.m_edge);
		  }
	  }


	  hacd::HaF64 m_area;
	  hacd::HaF64 m_perimeter;
};

dgMeshEffect::dgMeshEffect(const dgMeshEffect& source, hacd::HaF32 absoluteconcavity, hacd::HaI32 maxCount) 
	:dgPolyhedra()
{
	Init(true);

	dgMeshEffect mesh(source);
	hacd::HaI32 faceCount = mesh.GetTotalFaceCount() + 1;
	dgStack<dgClusterList> clusterPool(faceCount);
	dgClusterList* const clusters = &clusterPool[0];

	for (hacd::HaI32 i = 0; i < faceCount; i++) {
		clusters[i] = dgClusterList();
	}

	hacd::HaI32 meshMask = mesh.IncLRU();
	const dgBigVector* const points = mesh.m_points;

	// enumerate all faces, and initialize cluster pool
	dgMeshEffect::Iterator iter(mesh);

	hacd::HaI32 clusterIndex = 1;
	for (iter.Begin(); iter; iter++) {
		dgEdge* const edge = &(*iter);
		edge->m_userData = hacd::HaU64 (-1);
		if ((edge->m_mark != meshMask) && (edge->m_incidentFace > 0)) {
			hacd::HaF64 perimeter = hacd::HaF64(0.0f);
			dgEdge* ptr = edge;
			do {
				dgBigVector p1p0(points[ptr->m_incidentVertex] - points[ptr->m_prev->m_incidentVertex]);
				perimeter += sqrt(p1p0 % p1p0);
				ptr->m_incidentFace = clusterIndex;

				ptr->m_mark = meshMask;
				ptr = ptr->m_next;
			} while (ptr != edge);

			dgBigVector normal = mesh.FaceNormal(edge, &points[0][0], sizeof(dgBigVector));
			hacd::HaF64 mag = sqrt(normal % normal);

			dgClusterFace& faceInfo = clusters[clusterIndex].Append()->GetInfo();

			faceInfo.m_edge = edge;
			faceInfo.m_perimeter = perimeter;
			faceInfo.m_area = hacd::HaF64(0.5f) * mag;
			faceInfo.m_normal = normal.Scale(hacd::HaF64(1.0f) / mag);

			clusters[clusterIndex].m_perimeter = perimeter;
			clusters[clusterIndex].m_area = faceInfo.m_area;

			clusterIndex++;
		}
	}

	HACD_ASSERT(faceCount == clusterIndex);

	// recalculate all edge cost
	dgStack<hacd::HaI32> vertexMarksArray(mesh.GetVertexCount());
	dgStack<dgBigVector> vertexArray(mesh.GetVertexCount() * 2);
	
	dgBigVector* const vertexPool = &vertexArray[0];
	hacd::HaI32* const vertexMarks = &vertexMarksArray[0];
	memset(&vertexMarks[0], 0, vertexMarksArray.GetSizeInBytes());

	dgList<dgPairProxi> proxyList;
	dgUpHeap<dgList<dgPairProxi>::dgListNode*, hacd::HaF64> heap(mesh.GetCount() + 1000);

	hacd::HaI32 vertexMark = 0;

	hacd::HaF64 diagonalInv = hacd::HaF32(1.0f);
	hacd::HaF64 aspectRatioCoeficent = absoluteconcavity / hacd::HaF32(10.0f);
	meshMask = mesh.IncLRU();

	// calculate all the initial cost of all clusters, which at this time are all a single faces
	for (hacd::HaI32 faceIndex = 1; faceIndex < faceCount; faceIndex++) {
		vertexMark++;
		dgClusterList& clusterList = clusters[faceIndex];
		HACD_ASSERT(clusterList.GetFirst()->GetInfo().m_edge->m_incidentFace == faceIndex);
		clusterList.CalculateNodeCost(mesh, meshMask, &vertexPool[0], &vertexMarks[0], vertexMark, &clusters[0], diagonalInv, aspectRatioCoeficent, proxyList, heap);
	}
	
	
	// calculate all essential convex clusters by merging the all possible clusters according 
	// which combined concavity es lower that the max absolute concavity 
	// select the pair with the smaller concavity and fuse then into a larger cluster
	hacd::HaI32 essencialClustersCount = faceCount - 1;
	while (heap.GetCount() && ((heap.Value() < absoluteconcavity) || (essencialClustersCount > maxCount))) {
		dgList<dgPairProxi>::dgListNode* const pairNode = heap[0];
		heap.Pop();
		dgPairProxi& pair = pairNode->GetInfo();

		HACD_ASSERT((pair.m_edgeA && pair.m_edgeA) || (!pair.m_edgeA && !pair.m_edgeA));
		if (pair.m_edgeA && pair.m_edgeB) {

			HACD_ASSERT(pair.m_edgeA->m_incidentFace != pair.m_edgeB->m_incidentFace);

			// merge two clusters
			hacd::HaI32 faceIndexA = pair.m_edgeA->m_incidentFace;
			hacd::HaI32 faceIndexB = pair.m_edgeB->m_incidentFace;
			dgClusterList* listA = &clusters[faceIndexA];
			dgClusterList* listB = &clusters[faceIndexB];
			if (pair.m_edgeA->m_incidentFace > pair.m_edgeB->m_incidentFace) {
				Swap(faceIndexA, faceIndexB);
				Swap(listA, listB);
			}
			
			while (listB->GetFirst()) {
				dgClusterList::dgListNode* const nodeB = listB->GetFirst();
				listB->Unlink(nodeB);
				dgClusterFace& faceB = nodeB->GetInfo();

				dgEdge* ptr = faceB.m_edge;
				do {
					ptr->m_incidentFace = faceIndexA;
					ptr = ptr->m_next;
				} while (ptr != faceB.m_edge);
				listA->Append(nodeB);
			}
			essencialClustersCount --;

			listB->m_area = hacd::HaF32 (0.0f);
			listB->m_perimeter = hacd::HaF32 (0.0f);
			listA->m_area = pair.m_area;
			listA->m_perimeter = pair.m_perimeter;

			// recalculated the new metric for the new cluster, and tag the used cluster as invalid, so that 
			// other potential selection do not try merge with this this one, producing convex that re use a face more than once  
			hacd::HaI32 mark = mesh.IncLRU();
			for (dgClusterList::dgListNode* node = listA->GetFirst(); node; node = node->GetNext()) {
				dgClusterFace& face = node->GetInfo();
				dgEdge* ptr = face.m_edge;
				do {
					if (ptr->m_userData != hacd::HaU64 (-1)) {
						dgList<dgPairProxi>::dgListNode* const pairNode = (dgList<dgPairProxi>::dgListNode*) ptr->m_userData;
						dgPairProxi& pairProxy = pairNode->GetInfo();
						pairProxy.m_edgeA = NULL;
						pairProxy.m_edgeB = NULL;
					}
					ptr->m_userData = hacd::HaU64 (-1);
					ptr->m_twin->m_userData = hacd::HaU64 (-1);

					if ((ptr->m_twin->m_incidentFace == faceIndexA) || (ptr->m_twin->m_incidentFace < 0)) {
						ptr->m_mark = mark;
						ptr->m_twin->m_mark = mark;
					}

					if (ptr->m_mark != mark) {
						dgClusterList& adjacentList = clusters[ptr->m_twin->m_incidentFace];
						for (dgClusterList::dgListNode* adjacentNode = adjacentList.GetFirst(); adjacentNode; adjacentNode = adjacentNode->GetNext()) {
							dgClusterFace& adjacentFace = adjacentNode->GetInfo();
							dgEdge* adjacentEdge = adjacentFace.m_edge;
							do {
								if (adjacentEdge->m_twin->m_incidentFace == faceIndexA)	{
									adjacentEdge->m_twin->m_mark = mark;
								}
								adjacentEdge = adjacentEdge->m_next;
							} while (adjacentEdge != adjacentFace.m_edge);
						}
						ptr->m_mark = mark - 1;
					}
					ptr = ptr->m_next;
				} while (ptr != face.m_edge);
			}

			// re generated the cost of merging this new all its adjacent clusters, that are still alive. 
			vertexMark++;
			listA->CalculateNodeCost(mesh, mark, &vertexPool[0], &vertexMarks[0], vertexMark, &clusters[0], diagonalInv, aspectRatioCoeficent, proxyList, heap);
		}

		proxyList.Remove(pairNode);
	}


	// if the essential convex cluster count is larger than the the maximum specified by the user 
	// then resuming the cluster again to the heap and start merging then by the the worse merging criteria
	// also at this time add the distance heuristic to combine disjoint cluster as well.
	// this is where I disagree with Khaled Mamore, he uses a brute force approach by adding extra points.
	// I do this to the first partition and then connect disjoint face only on the perimeter
/*
maxCount = 1;
	while (essencialClustersCount > maxCount) {

		heap.Flush();
		meshMask = mesh.IncLRU();

		// color code each face on each cluster with it cluster index
		for (hacd::HaI32 faceIndex = 0; faceIndex < faceCount; faceIndex++) {
			dgClusterList& clusterList = clusters[faceIndex];
			if (clusterList.GetCount()) {
				for (dgClusterList::dgListNode* node = clusterList.GetFirst(); node; node = node->GetNext()) {
					dgClusterFace& face = node->GetInfo();
					dgEdge* ptr = face.m_edge;
					do {
						ptr->m_incidentFace = faceIndex;
						ptr = ptr->m_next;
					} while (ptr != face.m_edge);
				}
			}
		}
			
		for (hacd::HaI32 faceIndex = 0; faceIndex < faceCount; faceIndex++) {
			dgClusterList& clusterList = clusters[faceIndex];

			// note: add the disjoint cluster criteria here, but for now just ignore

			// calculate the cost again
			if (clusterList.GetCount()) {

				// note: something is wrong with my color coding that is not marking the perimeter corrently	

				vertexMark++;
				clusterList.CalculateNodeCost(mesh, meshMask, &vertexPool[0], &vertexMarks[0], vertexMark, &clusters[0], diagonalInv, aspectRatioCoeficent, proxyList, heap);	
			}
			
		}
	}
*/


	BeginPolygon();
	hacd::HaF32 layer = hacd::HaF32(0.0f);

	dgVertexAtribute polygon[256];
	memset(polygon, 0, sizeof(polygon));
	dgArray<dgBigVector> convexVertexBuffer(1024);
	for (hacd::HaI32 i = 0; i < faceCount; i++) {
		dgClusterList& clusterList = clusters[i];

		if (clusterList.GetCount())	{
			hacd::HaI32 count = 0;
			for (dgClusterList::dgListNode* node = clusterList.GetFirst(); node; node = node->GetNext()) {
				dgClusterFace& face = node->GetInfo();
				dgEdge* edge = face.m_edge;

				dgEdge* sourceEdge = source.FindEdge(edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
				do {
					hacd::HaI32 index = edge->m_incidentVertex;
					convexVertexBuffer[count] = points[index];

					count++;
					sourceEdge = sourceEdge->m_next;
					edge = edge->m_next;
				} while (edge != face.m_edge);
			}

			dgConvexHull3d convexHull(&convexVertexBuffer[0].m_x, sizeof(dgBigVector), count, 0.0);

			if (convexHull.GetCount()) {
				const dgBigVector* const vertex = convexHull.GetVertexPool();
				for (dgConvexHull3d::dgListNode* node = convexHull.GetFirst(); node; node = node->GetNext()) {
					const dgConvexHull3DFace* const face = &node->GetInfo();

					hacd::HaI32 i0 = face->m_index[0];
					hacd::HaI32 i1 = face->m_index[1];
					hacd::HaI32 i2 = face->m_index[2];

					polygon[0].m_vertex = vertex[i0];
					polygon[0].m_vertex.m_w = layer;

					polygon[1].m_vertex = vertex[i1];
					polygon[1].m_vertex.m_w = layer;

					polygon[2].m_vertex = vertex[i2];
					polygon[2].m_vertex.m_w = layer;

					AddPolygon(3, &polygon[0].m_vertex.m_x, sizeof(dgVertexAtribute), 0);
				}

				layer += hacd::HaF32(1.0f);
				//break;
			}
		}
	}
	EndPolygon(1.0e-5f);

	for (hacd::HaI32 i = 0; i < faceCount; i++) {
		clusters[i].RemoveAll();
	}
}


dgMeshEffect* dgMeshEffect::CreateConvexApproximation(hacd::HaF32 maxConcavity, hacd::HaI32 maxCount) const
{
	dgMeshEffect triangleMesh(*this);
	if (maxCount <= 1) {
		maxCount = 1;
	}
	if (maxConcavity <= hacd::HaF32 (1.0e-5f)) {
		maxConcavity = hacd::HaF32 (1.0e-5f);
	}
	dgMeshEffect* const convexPartion = HACD_NEW(dgMeshEffect)(triangleMesh, maxConcavity, maxCount );
	return convexPartion;
}
