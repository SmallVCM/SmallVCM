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

#include <Windows.h>
#undef max

//typedef PathTracer  t_Renderer;
//typedef VertexCM    t_Renderer;

int main(int argc, const char *argv[])
{
    int base_iterations = 10;
    if(argc > 1)
        base_iterations = atoi(argv[1]);

    Scene scene;
    scene.LoadCornellBox();

    //EyeLight renderer(scene.mCamera.mResolution);

    int numThreads = std::max(1, omp_get_num_procs()-1);
    omp_set_num_threads(numThreads);

    printf("Using %d threads\n", numThreads);
    DWORD startT = GetTickCount();

    {
        int iterations = base_iterations * 2;
        typedef PathTracer  t_Renderer;
        typedef AbstractRenderer* AbstractRendererPtr;
        AbstractRendererPtr *renderers;

        renderers = new AbstractRendererPtr[numThreads];

        for(int i=0; i<numThreads; i++)
            renderers[i] = new t_Renderer(scene, 1234 + i);

#pragma omp parallel for
        for(int iter=0; iter < iterations; iter++)
        {
            int threadId = omp_get_thread_num();
            renderers[threadId]->RunIteration(iter);
        }

        Framebuffer fbuffer;
        int usedRenderers = 0;
        for(int i=0; i<numThreads; i++)
        {
            if(!renderers[i]->WasUsed()) continue;
            if(usedRenderers == 0)
            {
                renderers[i]->GetFramebuffer(fbuffer);
            }
            else
            {
                Framebuffer tmp;
                renderers[i]->GetFramebuffer(tmp);
                fbuffer.Add(tmp);
            }
            usedRenderers++;
        }

        fbuffer.Scale(1.f / usedRenderers);

        fbuffer.SavePPM("cb_pt.ppm", 2.2f);
        //fbuffer.SavePFM("cb.pfm");
    }
    DWORD endT = GetTickCount();
    printf("Path tracing took %g s\n", float(endT - startT) / 1000.f);

    startT = GetTickCount();
    {
        int iterations = base_iterations;
        typedef VertexCM  t_Renderer;
        typedef AbstractRenderer* AbstractRendererPtr;
        AbstractRendererPtr *renderers;

        renderers = new AbstractRendererPtr[numThreads];

        for(int i=0; i<numThreads; i++)
            renderers[i] = new t_Renderer(scene, 1234 + i);

#pragma omp parallel for
        for(int iter=0; iter < iterations; iter++)
        {
            int threadId = omp_get_thread_num();
            renderers[threadId]->RunIteration(iter);
        }

        Framebuffer fbuffer;
        int usedRenderers = 0;
        for(int i=0; i<numThreads; i++)
        {
            if(!renderers[i]->WasUsed()) continue;
            if(usedRenderers == 0)
            {
                renderers[i]->GetFramebuffer(fbuffer);
            }
            else
            {
                Framebuffer tmp;
                renderers[i]->GetFramebuffer(tmp);
                fbuffer.Add(tmp);
            }
            usedRenderers++;
        }

        fbuffer.Scale(1.f / usedRenderers);

        fbuffer.SavePPM("cb_vcm.ppm", 2.2f);
        //fbuffer.SavePFM("cb.pfm");
    }

    endT = GetTickCount();
    printf("VCM took %g s\n", float(endT - startT) / 1000.f);

}