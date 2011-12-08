#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include	<float.h>

#include	"WuQuantizer.h"

#pragma warning(disable:4100)

using namespace hacd;

namespace HACD
{

hacd::HaU32	kmeans_cluster3d(const hacd::HaF32 *input,				// an array of input 3d data points.
		hacd::HaU32 inputSize,				// the number of input data points.
		hacd::HaU32 clumpCount,				// the number of clumps you wish to product.
		hacd::HaF32	*outputClusters,		// The output array of clumps 3d vectors, should be at least 'clumpCount' in size.
		hacd::HaU32	*outputIndices,			// A set of indices which remaps the input vertices to clumps; should be at least 'inputSize'
		hacd::HaF32 errorThreshold=0.01f,	// The error threshold to converge towards before giving up.
		hacd::HaF32 collapseDistance=0.01f); // distance so small it is not worth bothering to create a new clump.

template <class Type> struct Vec3d
{
	inline Type distanceSquared(const Vec3d &a) const
	{
		Type dx = x-a.x;
		Type dy = y-a.y;
		Type dz = z-a.z;
		return dx*dx+dy*dy+dz*dz;
	}
	inline void operator+=(const Vec3d &v)
	{
		x+=v.x;
		y+=v.y;
		z+=v.z;
	}
	inline void operator*=(const Type v)
	{
		x*=v;
		y*=v;
		z*=v;
	}
	inline void zero(void) { x = 0; y = 0; z = 0; };

	Type x;
	Type y;
	Type z;
};

template <class Vec,class Type >
hacd::HaU32	kmeans_cluster(const Vec *input,
						   hacd::HaU32 inputCount,
						   hacd::HaU32 clumpCount,
						   Vec *clusters,
						   hacd::HaU32 *outputIndices,
						   Type threshold, // controls how long it works to converge towards a least errors solution.
						   Type collapseDistance) // distance between clumps to consider them to be essentially equal.
{
	hacd::HaU32 convergeCount = 64; // maximum number of iterations attempting to converge to a solution..
	hacd::HaU32 *counts = (hacd::HaU32 *)HACD_ALLOC(sizeof(hacd::HaU32)*clumpCount);
	Type error=0;
	if ( inputCount <= clumpCount ) // if the number of input points is less than our clumping size, just return the input points.
	{
		clumpCount = inputCount;
		for (hacd::HaU32 i=0; i<inputCount; i++)
		{
			if ( outputIndices )
			{
				outputIndices[i] = i;
			}
			clusters[i] = input[i];
			counts[i] = 1;
		}
	}
	else
	{
		Vec *centroids = (Vec *)HACD_ALLOC(sizeof(Vec)*clumpCount);

		// Take a sampling of the input points as initial centroid estimates.
		for (hacd::HaU32 i=0; i<clumpCount; i++)
		{
			hacd::HaU32 index = (i*inputCount)/clumpCount;
			assert( index >= 0 && index < inputCount );
			clusters[i] = input[index];
		}

		// Here is the main convergence loop
		Type old_error = FLT_MAX;	// old and initial error estimates are max Type
		error = FLT_MAX;
		do
		{
			old_error = error;	// preserve the old error
			// reset the counts and centroids to current cluster location
			for (hacd::HaU32 i=0; i<clumpCount; i++)
			{
				counts[i] = 0;
				centroids[i].zero();
			}
			error = 0;
			// For each input data point, figure out which cluster it is closest too and add it to that cluster.
			for (hacd::HaU32 i=0; i<inputCount; i++)
			{
				Type min_distance = FLT_MAX;
				// find the nearest clump to this point.
				for (hacd::HaU32 j=0; j<clumpCount; j++)
				{
					Type distance = input[i].distanceSquared( clusters[j] );
					if ( distance < min_distance )
					{
						min_distance = distance;
						outputIndices[i] = j; // save which clump this point indexes
					}
				}
				hacd::HaU32 index = outputIndices[i]; // which clump was nearest to this point.
				centroids[index]+=input[i];
				counts[index]++;	// increment the counter indicating how many points are in this clump.
				error+=min_distance; // save the error accumulation
			}
			// Now, for each clump, compute the mean and store the result.
			for (hacd::HaU32 i=0; i<clumpCount; i++)
			{
				if ( counts[i] ) // if this clump got any points added to it...
				{
					Type recip = 1.0f / (Type)counts[i];	// compute the average (center of those points)
					centroids[i]*=recip;	// compute the average center of the points in this clump.
					clusters[i] = centroids[i]; // store it as the new cluster.
				}
			}
			// decrement the convergence counter and bail if it is taking too long to converge to a solution.
			convergeCount--;
			if (convergeCount == 0 )
			{
				break;
			}
			if ( error < threshold ) // early exit if our first guess is already good enough (if all input points are the same)
				break;
		} while ( fabs(error - old_error) > threshold ); // keep going until the error is reduced by this threshold amount.

		HACD_FREE(centroids);
	}

	// ok..now we prune the clumps if necessary.
	// The rules are; first, if a clump has no 'counts' then we prune it as it's unused.
	// The second, is if the centroid of this clump is essentially  the same (based on the distance tolerance)
	// as an existing clump, then it is pruned and all indices which used to point to it, now point to the one
	// it is closest too.
	hacd::HaU32 outCount = 0; // number of clumps output after pruning performed.
	Type d2 = collapseDistance*collapseDistance; // squared collapse distance.
	for (hacd::HaU32 i=0; i<clumpCount; i++)
	{
		if ( counts[i] == 0 ) // if no points ended up in this clump, eliminate it.
			continue;
		// see if this clump is too close to any already accepted clump.
		bool add = true;
		hacd::HaU32 remapIndex = outCount; // by default this clump will be remapped to its current index.
		for (hacd::HaU32 j=0; j<outCount; j++)
		{
			Type distance = clusters[i].distanceSquared(clusters[j]);
			if ( distance < d2 )
			{
				remapIndex = j;
				add = false; // we do not add this clump
				break;
			}
		}
		// If we have fewer output clumps than input clumps so far, then we need to remap the old indices to the new ones.
		if ( outputIndices )
		{
			if ( outCount != i || !add ) // we need to remap indices!  everything that was index 'i' now needs to be remapped to 'outCount'
			{
				for (hacd::HaU32 j=0; j<inputCount; j++)
				{
					if ( outputIndices[j] == i )
					{
						outputIndices[j] = remapIndex; //
					}
				}
			}
		}
		if ( add )
		{
			clusters[outCount] = clusters[i];
			outCount++;
		}
	}
	HACD_FREE(counts);
	clumpCount = outCount;
	return clumpCount;
};

hacd::HaU32	kmeans_cluster3d(const hacd::HaF32 *input,				// an array of input 3d data points.
							 hacd::HaU32 inputSize,				// the number of input data points.
							 hacd::HaU32 clumpCount,				// the number of clumps you wish to produce
							 hacd::HaF32	*outputClusters,		// The output array of clumps 3d vectors, should be at least 'clumpCount' in size.
							 hacd::HaU32	*outputIndices,			// A set of indices which remaps the input vertices to clumps; should be at least 'inputSize'
							 hacd::HaF32 errorThreshold,	// The error threshold to converge towards before giving up.
							 hacd::HaF32 collapseDistance) // distance so small it is not worth bothering to create a new clump.
{
	const Vec3d< hacd::HaF32 > *_input = (const Vec3d<hacd::HaF32> *)input;
	Vec3d<hacd::HaF32> *_output = (Vec3d<hacd::HaF32> *)outputClusters;
	return kmeans_cluster< Vec3d<hacd::HaF32>, hacd::HaF32 >(_input,inputSize,clumpCount,_output,outputIndices,errorThreshold,collapseDistance);
}


#if 0


typedef unsigned char	byte;
typedef unsigned int	uint;
typedef unsigned long	ulong;
//typedef int				bool;

#ifndef True
#define False	0
#define True	1
#endif

//
// RGBType is a simple 8-bit color triple
//
typedef struct {
	byte    r,g,b;                      // The color
} RGBType;

//
// OctnodeType is a generic octree node
//
typedef struct _octnode {
	int     level;                      // Level for this node
	bool    isleaf;                     // TRUE if this is a leaf node
	byte    index;                      // Color table index
	ulong   npixels;                    // Total pixels that have this color
	ulong   redsum, greensum, bluesum;  // Sum of the color components
	RGBType *color;                     // Color at this (leaf) node
	struct _octnode *child[8];          // Tree pointers
	struct _octnode *nextnode;          // Reducible list pointer
} OctreeType;

OctreeType *CreateOctNode(int level);
void MakePaletteTable(OctreeType *tree, RGBType table[], int *index);
ulong TotalLeafNodes(void);
void ReduceTree(void);
void InsertTree(OctreeType **tree, RGBType *color, uint depth);
int QuantizeColor(OctreeType *tree, RGBType *color);



#define		COLORBITS	8
#define		TREEDEPTH	6

byte MASK[COLORBITS] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
#define BIT(b,n)    (((b)&MASK[n])>>n)
#define LEVEL(c,d)  ((BIT((c)->r,(d)))<<2 | BIT((c)->g,(d))<<1 | BIT((c)->b,(d)))


OctreeType  *reducelist[TREEDEPTH];             // List of reducible nodes

static byte glbLeafLevel = TREEDEPTH;
static uint glbTotalLeaves = 0;

static void MakeReducible(int level, OctreeType *node);
static OctreeType *GetReducible(void);

//----------------------------------------------------------------------------
//
// InsertTree
//
// Insert a color into the octree
//
void InsertTree(OctreeType **tree, RGBType *color, uint depth)
{
//    int level;

    if (*tree == (OctreeType *)NULL) {
        *tree = CreateOctNode(depth);
	}
    if ((*tree)->isleaf) {
        (*tree)->npixels++;
        (*tree)->redsum += color->r;
        (*tree)->greensum += color->g;
        (*tree)->bluesum += color->b;
	}
	else {
		InsertTree(&((*tree)->child[LEVEL(color, TREEDEPTH-depth)]),
				   color,
				   depth+1);
	}
}



//--------------------------------------------------------------------------
//
// ReduceTree
//
// Combines all the children of a node into the parent, makes the parent
// into a leaf
//
void ReduceTree()
{
	OctreeType  *node;
	ulong   sumred=0, sumgreen=0, sumblue=0;
	byte    i, nchild=0;

	node = GetReducible();
    for (i = 0; i < COLORBITS; i++) {
		if (node->child[i]) {
			nchild++;
			sumred += node->child[i]->redsum;
			sumgreen += node->child[i]->greensum;
			sumblue += node->child[i]->bluesum;
			node->npixels += node->child[i]->npixels;
			free(node->child[i]);
		}
	}
	node->isleaf = True;
	node->redsum = sumred;
	node->greensum = sumgreen;
	node->bluesum = sumblue;
    glbTotalLeaves -= (nchild - 1);
}




//--------------------------------------------------------------------------
//
// CreateOctNode
//
// Allocates and initializes a new octree node.  The level of the node is
// determined by the caller.
//
// Arguments:
//  level   int     Tree level where the node will be inserted.
//
// Returns:
//  Pointer to newly allocated node.  Does not return on failure.
//
OctreeType *CreateOctNode(int level)
{
	static OctreeType  *newnode;
    int                 i;

    newnode = (OctreeType *)HACD_ALLOC(sizeof(OctreeType));
    newnode->level = level;
	newnode->isleaf = level == glbLeafLevel;
    if (newnode->isleaf) {
        glbTotalLeaves++;
    }
    else {
        MakeReducible(level, newnode);
    }
    newnode->npixels = 0;
	newnode->index = 0;
    newnode->npixels = 0;
	newnode->redsum = newnode->greensum = newnode->bluesum = 0L;
    for (i = 0; i < COLORBITS; i++) {
        newnode->child[i] = NULL;
	}
	return newnode;
}


//-----------------------------------------------------------------------
//
// MakeReducible
//
// Adds a node to the reducible list for the specified level
//
static void MakeReducible(int level, OctreeType *node)
{
    node->nextnode = reducelist[level];
    reducelist[level] = node;
}


//-----------------------------------------------------------------------
//
// GetReducible
//
// Returns the next available reducible node at the tree's leaf level
//
static OctreeType *GetReducible(void)
{
    OctreeType *node;
    
    while (reducelist[glbLeafLevel-1] == NULL) {
        glbLeafLevel--;
    }
    node = reducelist[glbLeafLevel-1];
    reducelist[glbLeafLevel-1] = reducelist[glbLeafLevel-1]->nextnode;
	return node;
}



//---------------------------------------------------------------------------
//
// MakePaletteTable
//
// Given a color octree, traverse the tree and do the following:
//	- Add the averaged RGB leaf color to the color palette table;
//	- Store the palette table index in the tree;
//
// When this recursive function finally returns, 'index' will contain
// the total number of colors in the palette table.
//
void MakePaletteTable(OctreeType *tree, RGBType table[], int *index)
{
	int	i;

	if (tree->isleaf) {
		table[*index].r = (byte)(tree->redsum / tree->npixels);
        table[*index].g = (byte)(tree->greensum / tree->npixels);
        table[*index].b = (byte)(tree->bluesum / tree->npixels);
		tree->index = (byte)*index;
		(*index)++;
	}
	else {
        for (i = 0; i < COLORBITS; i++) {
			if (tree->child[i]) {
				MakePaletteTable(tree->child[i], table, index);
			}
		}
	}
}


//---------------------------------------------------------------------------
//
// QuantizeColor
//
// Returns the palette table index of an RGB color by traversing the
// octree to the leaf level
//
int QuantizeColor(OctreeType *tree, RGBType *color)
{
	if (tree->isleaf) {
		return tree->index;
	}
	else {
        QuantizeColor(tree->child[LEVEL(color,6-tree->level)],color);
	}
	return 0;
}



//---------------------------------------------------------------------------
//
// TotalLeafNodes
//
// Returns the total leaves in the tree (glbTotalLeaves)
//
ulong TotalLeafNodes()
{
	return glbTotalLeaves;
}
#endif


#define MAX_OCTREE_DEPTH 6

class MyWuQuantizer : public WuQuantizer, public hacd::UserAllocated
{
public:
	class Vec3
	{
	public:

		Vec3(HaF32 _x,HaF32 _y,HaF32 _z)
		{
			x = _x;
			y = _y;
			z = _z;
		}

		Vec3(void)
		{
		}

		Vec3(const hacd::HaF32 *p)
		{
			x = p[0];
			y = p[1];
			z = p[2];
		}

		HaF32	x;
		HaF32	y;
		HaF32	z;
	};

    class OctreeNode : public UserAllocated
    {
    public:
    	OctreeNode(HaU32 level)
    	{
    		mLevel 		= level;
    		mIsLeaf		= level==MAX_OCTREE_DEPTH;
    		mVertCount	= 0;
    		mSumX		= 0;
    		mSumY		= 0;
    		mSumZ		= 0;
    		mValue		= Vec3(0,0,0);
    		for (HaU32 i=0; i<8; i++)
    		{
    		    mChildren[i] = NULL;
    		}
    		mNextNode = NULL;
    	}
    	bool		mIsLeaf;			// whether or  not this is a leaf node.
    	HaU32		mLevel;				// level of the octree node. High bit indicates it is a leaf node.
    	HaU32		mVertCount;			// number of vertices with this value.
    	HaU32		mSumX;				// total sum of quantized X axis value.
    	HaU32		mSumY;				// total sum of quantized Y axis value.
    	HaU32		mSumZ;				// total sum of quantized Z axis value.
    	Vec3		mValue;			// resultant value.
    	OctreeNode	*mChildren[8];		// the 8 children.
    	OctreeNode	*mNextNode;			// reducible list pointer.
    };



	typedef STDNAME::vector< Vec3 > Vec3Vector;

	MyWuQuantizer(void)
	{
		mScale = Vec3(1,1,1);
		mCenter = Vec3(0,0,0);
		for (HaU32 i=0; i<MAX_OCTREE_DEPTH; i++)
		{
			mReduceList[i] = NULL;
		}
	}

	// use the Wu quantizer with 10 bits of resolution on each axis.  Precision down to 0.0009765625
	// All input data is normalized to a unit cube.

	virtual const hacd::HaF32 * wuQuantize3D(hacd::HaU32 vcount,
		const hacd::HaF32 *vertices,
		bool denormalizeResults,
		hacd::HaU32 maxVertices,
		hacd::HaU32 &outputCount)
	{
		const hacd::HaF32 *ret = NULL;
		outputCount = 0;

		normalizeInput(vcount,vertices);

		//
		for (HaU32 i=0; i<MAX_OCTREE_DEPTH; i++)
		{
			mReduceList[i] = NULL;
		}


		return ret;
	}

	// Use the kemans quantizer, similar results, but much slower.
	virtual const hacd::HaF32 * kmeansQuantize3D(hacd::HaU32 vcount,
		const hacd::HaF32 *vertices,
		bool denormalizeResults,
		hacd::HaU32 maxVertices,
		hacd::HaU32 &outputCount)
	{
		const hacd::HaF32 *ret = NULL;
		outputCount = 0;
		mNormalizedInput.clear();
		mQuantizedOutput.clear();

		if ( vcount > 0 )
		{
			normalizeInput(vcount,vertices);
			HaF32 *quantizedOutput = (HaF32 *)HACD_ALLOC( sizeof(HaF32)*3*vcount);
			outputCount = kmeans_cluster3d(&mNormalizedInput[0].x, vcount, maxVertices, quantizedOutput, NULL, 0.01f, 0.0001f );
			if ( outputCount > 0 )
			{
				if ( denormalizeResults )
				{
					for (HaU32 i=0; i<outputCount; i++)
					{
						Vec3 v( &quantizedOutput[i*3] );
						v.x = v.x*mScale.x + mCenter.x;
						v.y = v.x*mScale.y + mCenter.y;
						v.z = v.x*mScale.z + mCenter.z;
						mQuantizedOutput.push_back(v);
					}
				}
				else
				{
					for (HaU32 i=0; i<outputCount; i++)
					{
						Vec3 v( &quantizedOutput[i*3] );
						mQuantizedOutput.push_back(v);
					}
				}
				HACD_FREE(quantizedOutput);
				ret = &mQuantizedOutput[0].x;
			}
		}

		return ret;
	}

	virtual void release(void)
	{
		delete this;
	}

	virtual const hacd::HaF32 * getDenormalizeScale(void) const 
	{
		return &mScale.x;
	}

	virtual const hacd::HaF32 * getDenormalizeCenter(void) const
	{
		return &mCenter.x;
	}



private:

	void normalizeInput(HaU32 vcount,const HaF32 *vertices)
	{
		mNormalizedInput.clear();
		mQuantizedOutput.clear();
		Vec3 bmin(vertices);
		Vec3 bmax(vertices);
		for (HaU32 i=1; i<vcount; i++)
		{
			Vec3 v(&vertices[i*3]);

			if ( v.x < bmin.x ) 
			{
				bmin.x = v.x;
			}
			else if ( v.x > bmax.x )
			{
				bmax.x = v.x;
			}

			if ( v.y < bmin.y ) 
			{
				bmin.y = v.y;
			}
			else if ( v.y > bmax.y )
			{
				bmax.y = v.y;
			}

			if ( v.z < bmin.z ) 
			{
				bmin.z = v.z;
			}
			else if ( v.z > bmax.z )
			{
				bmax.z = v.z;
			}
		}

		mCenter.x = (bmin.x+bmax.x)*0.5f;
		mCenter.y = (bmin.y+bmax.y)*0.5f;
		mCenter.z = (bmin.z+bmax.z)*0.5f;

		HaF32 dx = bmax.x-bmin.x;
		HaF32 dy = bmax.y-bmin.y;
		HaF32 dz = bmax.z-bmin.z;

		if ( dx == 0 )
		{
			mScale.x = 1;
		}
		else
		{
			dx = dx*1.001f;
			mScale.x = dx*0.5f;
		}
		if ( dy == 0 )
		{
			mScale.y = 1;
		}
		else
		{
			dy = dy*1.001f;
			mScale.y = dy*0.5f;
		}
		if ( dz == 0 )
		{
			mScale.z = 1;
		}
		else
		{
			dz = dz*1.001f;
			mScale.z = dz*0.5f;
		}

		Vec3 recip;
		recip.x = 1.0f / mScale.x;
		recip.y = 1.0f / mScale.y;
		recip.z = 1.0f / mScale.z;

		for (HaU32 i=0; i<vcount; i++)
		{
			Vec3 v(&vertices[i*3]);

			v.x = (v.x-mCenter.x)*recip.x;
			v.y = (v.y-mCenter.y)*recip.y;
			v.z = (v.z-mCenter.z)*recip.z;

			HACD_ASSERT( v.x >= -1 && v.x <= 1 );
			HACD_ASSERT( v.y >= -1 && v.y <= 1 );
			HACD_ASSERT( v.z >= -1 && v.z <= 1 );
			mNormalizedInput.push_back(v);
		}
	}

	virtual ~MyWuQuantizer(void)
	{

	}

	Vec3		mScale;
	Vec3		mCenter;
	Vec3Vector	mNormalizedInput;
	Vec3Vector	mQuantizedOutput;

	OctreeNode	*mReduceList[MAX_OCTREE_DEPTH];

};

WuQuantizer * createWuQuantizer(void)
{
	MyWuQuantizer *m = HACD_NEW(MyWuQuantizer);
	return static_cast< WuQuantizer *>(m);
}


}; // end of HACD namespace
