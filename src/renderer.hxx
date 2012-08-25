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
    AbstractRenderer(const Scene& aScene) : mScene(aScene)
    {
        mMaxPathLength = 10;
        mIterations = 0;
        mFramebuffer.Setup(aScene.mCamera.mResolution);
    }

    virtual void RunIteration(int aIteration) = 0;

    void GetFramebuffer(Framebuffer& oFramebuffer)
    {
        oFramebuffer = mFramebuffer;
        if(mIterations > 0)
            oFramebuffer.Scale(1.f / mIterations);
    }

    //! Whether this renderer was used at all
    bool WasUsed() const { return mIterations > 0; }
public:
    uint         mMaxPathLength;
protected:
    int          mIterations;
    Framebuffer  mFramebuffer;
    const Scene& mScene;
};

#endif //__RENDERER_HXX__
