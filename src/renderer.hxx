/*
 * This is published under Apache 2.0
 */

#ifndef __RENDERER_HXX__
#define __RENDERER_HXX__

#include <vector>
#include <cmath>
#include "scene.hxx"
#include "framebuffer.hxx"

class AbstractRenderer
{
public:
    AbstractRenderer()
    {
        mMaxPathLength = 2;
    }

    virtual void RunIteration(int aIteration, const Scene& aScene) = 0;
    virtual void GetFramebuffer(Framebuffer& oFramebuffer) = 0;
protected:
    uint mMaxPathLength;
};

#endif //__RENDERER_HXX__
