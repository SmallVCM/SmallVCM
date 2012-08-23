/*
 * This is published under Apache 2.0
 */

#include <vector>
#include <cmath>
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

//typedef PathTracer  t_Renderer;
typedef VertexCM    t_Renderer;

int main(int argc, const char *argv[])
{
    int iterations = 100;
    if(argc > 1)
        iterations = atoi(argv[1]);

    Scene scene;
    scene.LoadCornellBox();

    //EyeLight renderer(scene.mCamera.mResolution);

    int numThreads = std::max(1, omp_get_num_procs()-1);
    omp_set_num_threads(numThreads);

    printf("Using %d threads\n", numThreads);

    typedef AbstractRenderer* AbstractRendererPtr;
    AbstractRendererPtr *renderers;

    renderers = new AbstractRendererPtr[numThreads];

    for(int i=0; i<numThreads; i++)
        renderers[i] = new t_Renderer(scene.mCamera.mResolution, 1234 + i);

#pragma omp parallel for
    for(int iter=0; iter < iterations; iter++)
    {
        int threadId = omp_get_thread_num();
        renderers[threadId]->RunIteration(iter, scene);
    }

    Framebuffer fbuffer;
    renderers[0]->GetFramebuffer(fbuffer);
    for(int i=1; i<numThreads; i++)
    {
        Framebuffer tmp;
        renderers[i]->GetFramebuffer(tmp);
        fbuffer.Add(tmp);
    }

    fbuffer.Scale(1.f / numThreads);

    fbuffer.SavePPM("cb.ppm", 2.2f);
    fbuffer.SavePFM("cb.pfm");
}