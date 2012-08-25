/*
 * This is published under Apache 2.0
 */

#include <vector>
#include <cmath>
#include <time.h>
#include "math.hxx"
#include "ray.hxx"
#include "geometry.hxx"
#include "camera.hxx"
#include "framebuffer.hxx"
#include "scene.hxx"
#include "eyelight.hxx"
#include "pathtracer.hxx"
#include "bxdf.hxx"
#include "vertexcm.hxx"

#include <omp.h>

struct Config
{
    enum Algorithm
    {
        kEyeLight,
        kPathTracing,
        kLightTracing,
        kProgressivePhotonMapping,
        kBidirectionalPhotonMapping,
        kBidirectionalPathTracing,
        kVertexConnectionMerging
    };

    const Scene *mScene;
    Algorithm   mAlgorithm;
    int         mIterations;
    Framebuffer *mFramebuffer;
    int         mNumThreads;
    int         mBaseSeed;
    uint        mMaxPathLength;
};


float render(const Config &aConfig)
{
    omp_set_num_threads(aConfig.mNumThreads);

    typedef AbstractRenderer* AbstractRendererPtr;
    AbstractRendererPtr *renderers;
    renderers = new AbstractRendererPtr[aConfig.mNumThreads];

    switch(aConfig.mAlgorithm)
    {
    case Config::kEyeLight:
        printf("Using EyeLight (L.N, DotLN)\n");
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new EyeLight(*aConfig.mScene);
        break;
    case Config::kPathTracing:
        printf("Using Path Tracing\n");
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new PathTracer(*aConfig.mScene,
            aConfig.mBaseSeed + i);
        break;
    case Config::kLightTracing:
        printf("Using Light Tracing\n");
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(*aConfig.mScene,
            VertexCM::Lighttrace, aConfig.mBaseSeed + i);
        break;
    case Config::kProgressivePhotonMapping:
        printf("Using Progressive Photon Mapping\n");
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(*aConfig.mScene,
            VertexCM::Ppm, aConfig.mBaseSeed + i);
        break;
    case Config::kBidirectionalPhotonMapping:
        printf("Using Bidirectional Photon Mapping\n");
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(*aConfig.mScene,
            VertexCM::Bpm, aConfig.mBaseSeed + i);
        break;
    case Config::kBidirectionalPathTracing:
        printf("Using Bidirectional Path Tracing\n");
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(*aConfig.mScene,
            VertexCM::Bpt, aConfig.mBaseSeed + i);
        break;
    case Config::kVertexConnectionMerging:
        printf("Using Vertex Connection Merging\n");
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(*aConfig.mScene,
            VertexCM::Vcm, aConfig.mBaseSeed + i);
        break;
    }

    for(int i=0; i<aConfig.mNumThreads; i++)
        renderers[i]->mMaxPathLength = aConfig.mMaxPathLength;

    clock_t startT = clock();
#pragma omp parallel for
    for(int iter=0; iter < aConfig.mIterations; iter++)
    {
        int threadId = omp_get_thread_num();
        renderers[threadId]->RunIteration(iter);
    }
    clock_t endT = clock();

    int usedRenderers = 0;
    for(int i=0; i<aConfig.mNumThreads; i++)
    {
        if(!renderers[i]->WasUsed()) continue;
        if(usedRenderers == 0)
        {
            renderers[i]->GetFramebuffer(*aConfig.mFramebuffer);
        }
        else
        {
            Framebuffer tmp;
            renderers[i]->GetFramebuffer(tmp);
            aConfig.mFramebuffer->Add(tmp);
        }
        usedRenderers++;
    }

    aConfig.mFramebuffer->Scale(1.f / usedRenderers);

    for(int i=0; i<aConfig.mNumThreads; i++)
        delete renderers[i];
    delete [] renderers;

    return float(endT - startT) / CLOCKS_PER_SEC;
}

int main(int argc, const char *argv[])
{
    int base_iterations = 1;
    if(argc > 1)
        base_iterations = atoi(argv[1]);

    const int numThreads = std::max(1, omp_get_num_procs()-1);
    printf("Using %d threads\n", numThreads);

    Scene scene;
    scene.LoadCornellBox(Vec2i(256));
    scene.BuildSceneSphere();
    Framebuffer fbuffer;

    Config config;
    config.mScene         = &scene;
    config.mAlgorithm     = Config::kVertexConnectionMerging;
    config.mIterations    = 10;
    config.mFramebuffer   = &fbuffer;
    config.mNumThreads    = numThreads;
    config.mBaseSeed      = 1234;
    config.mMaxPathLength = 10;

    float time = render(config);

    printf("Path tracing took %g s\n", time);
    fbuffer.SaveBMP("cb_pt.bmp", 2.2f);
    fbuffer.SavePPM("cb_pt.ppm", 2.2f);
}