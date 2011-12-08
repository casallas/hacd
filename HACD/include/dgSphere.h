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

#ifndef __dgSphere__
#define __dgSphere__

//#include "dgStdafx.h"
#include "dgTypes.h"
#include "dgVector.h"
#include "dgMatrix.h"
#include "dgQuaternion.h"


//class dgFace; 
class dgPlane;
//class dgCamera;

class dgSphere: public dgMatrix
{
	public:
	dgSphere ();
//	dgSphere (const dgQuaternion &quat, const dgVector &position, const dgVector dim = dgVector(0, 0, 0, 0));
//	dgSphere (const dgMatrix &matrix, const dgVector dim = dgVector(0, 0, 0, 0));

	dgSphere (const dgQuaternion &quat, const dgVector &position, const dgVector& dim = dgVector(0, 0, 0, 0));
	dgSphere (const dgMatrix &matrix, const dgVector& dim = dgVector(0, 0, 0, 0));

	dgSphere &operator= (const dgMatrix &arg);
	void Scale (hacd::HaF32 Ws, hacd::HaF32 Hs, hacd::HaF32 Bs) ;
	void SetDimensions (hacd::HaF32 W, hacd::HaF32 H, hacd::HaF32 B);
	void SetDimensions (const hacd::HaF32 vertex[], hacd::HaI32 strideInBytes, hacd::HaI32 vertexCount, const dgMatrix *basis = NULL);
	void SetDimensions (const hacd::HaF32 vertex[], hacd::HaI32 strideInBytes, const hacd::HaI32 trangles[], hacd::HaI32 indexCount, const dgMatrix *basis);
//	void SetDimensions (const hacd::HaF32 vertex[], hacd::HaI32 strideInBytes, const hacd::HaI32 index[], hacd::HaI32 indexCount, const dgMatrix *basis = NULL);

	// return:  0 if the sphere is wholly inside the viewport
	//          1 if the sphere is partially inside the viewport
	//         -1 if the sphere is wholly outside the viewport
//	hacd::HaI32 VisibilityTest (const dgCamera* camera) const;
//	hacd::HaI32 VisibilityTest (const dgCamera* camera, const dgMatrix &worldMatrix) const; 
//	void Render (const dgCamera* camera, const dgMatrix &transform, unsigned rgb) const;

	private:
/*
 	typedef hacd::HaI32 (dgSphere::*CachedVisibilityTest) (const dgMatrix &point, const dgPlane* plane) const;

	mutable CachedVisibilityTest planeTest;
	static CachedVisibilityTest planeTestArray[6];

	void ChangeCachedVisibilityTest (CachedVisibilityTest	fnt);
	hacd::HaI32 FrontTest  (const dgMatrix &point, const dgPlane* plane) const;
	hacd::HaI32 RearTest   (const dgMatrix &point, const dgPlane* plane) const;
	hacd::HaI32 LeftTest   (const dgMatrix &point, const dgPlane* plane) const;
	hacd::HaI32 RightTest  (const dgMatrix &point, const dgPlane* plane) const;
	hacd::HaI32 TopTest    (const dgMatrix &point, const dgPlane* plane) const;
	hacd::HaI32 BottomTest (const dgMatrix &point, const dgPlane* plane) const;
	hacd::HaI32 VisibilityTestLow (const dgCamera* camera, const dgMatrix& viewMNatrix) const;
*/

	public:
	dgVector m_size;
}; 


const dgSphere& GetIdentitySphere();


inline dgSphere &dgSphere::operator= (const dgMatrix &arg)
{
	m_front = arg.m_front;
	m_up = arg.m_up;
	m_right = arg.m_right;
	m_posit = arg.m_posit;
	return *this;
}

inline void dgSphere::SetDimensions (hacd::HaF32 W, hacd::HaF32 H, hacd::HaF32 B)
{
	m_size = dgVector (dgAbsf(W), dgAbsf(H), dgAbsf(B), dgSqrt (W * W + H * H + B * B));
}

inline void dgSphere::Scale (hacd::HaF32 Ws, hacd::HaF32 Hs, hacd::HaF32 Bs) 
{
	SetDimensions (m_size.m_x * Ws, m_size.m_y * Hs, m_size.m_z * Bs);
}


#endif

