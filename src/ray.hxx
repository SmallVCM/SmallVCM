/*
 * This is published under Apache 2.0
 */

#ifndef __RAY_HXX__
#define __RAY_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"

//////////////////////////////////////////////////////////////////////////
// Ray casting
struct Ray
{
    Vec3f org;  //!< Ray origin
    Vec3f dir;  //!< Ray direction
    float tmin; //!< Minimal distance to intersection
};

struct Isect
{
    float dist;    //!< Distance to closest intersection (serves as ray.tmax)
    int   matID;   //!< ID of intersected material
    int   lightID; //!< ID of intersected light (if < 0, then none)
    Vec3f normal;  //!< Normal at the intersection
};

#endif //__RAY_HXX__
