#ifndef WU_QUANTIZER_H
#define WU_QUANTIZER_H

#include "PlatformConfigHACD.h"

namespace HACD
{

class WuQuantizer
{
public:

	// use the Wu quantizer with 10 bits of resolution on each axis.  Precision down to 0.0009765625
	// All input data is normalized to a unit cube.
	virtual const hacd::HaF32 * wuQuantize3D(hacd::HaU32 vcount,
											const hacd::HaF32 *vertices,
											bool denormalizeResults,
											hacd::HaU32 maxVertices,
											hacd::HaU32 &outputCount) = 0;

	// Use the kemans quantizer, similar results, but much slower.
	virtual const hacd::HaF32 * kmeansQuantize3D(hacd::HaU32 vcount,
												const hacd::HaF32 *vertices,
												bool denormalizeResults,
												hacd::HaU32 maxVertices,
												hacd::HaU32 &outputCount) = 0;

	virtual const hacd::HaF32 * getDenormalizeScale(void) const = 0;

	virtual const hacd::HaF32 * getDenormalizeCenter(void) const = 0;

	virtual void release(void) = 0;


protected:
	virtual ~WuQuantizer(void)
	{

	}
};

WuQuantizer * createWuQuantizer(void);

};

#endif
