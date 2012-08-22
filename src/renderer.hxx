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
    virtual void RunIteration(const Scene& aScene) = 0;
    virtual void GetFramebuffer(Framebuffer& oFramebuffer) = 0;
};

#endif //__RENDERER_HXX__