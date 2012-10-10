/*
 * Copyright (C) 2012, Tomas Davidovic (http://www.davidovic.cz)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * (The above is MIT License: http://en.wikipedia.org/wiki/MIT_License)
 */

#include <vector>
#include <cmath>
#include <time.h>
#include <cstdlib>
#include "math.hxx"
#include "ray.hxx"
#include "geometry.hxx"
#include "camera.hxx"
#include "framebuffer.hxx"
#include "scene.hxx"
#include "eyelight.hxx"
#include "pathtracer.hxx"
#include "bsdf.hxx"
#include "vertexcm.hxx"
#include "html_writer.hxx"
#include "config.hxx"

#include <omp.h>
#include <string>
#include <set>
#include <sstream>

//////////////////////////////////////////////////////////////////////////
// The main rendering function, renders what is in aConfig

float render(
    const Config &aConfig,
    int *oUsedIterations = NULL)
{
    // Set number of used threads
    omp_set_num_threads(aConfig.mNumThreads);

    // Create 1 renderer per thread
    typedef AbstractRenderer* AbstractRendererPtr;
    AbstractRendererPtr *renderers;
    renderers = new AbstractRendererPtr[aConfig.mNumThreads];

    for(int i=0; i<aConfig.mNumThreads; i++)
    {
        renderers[i] = CreateRenderer(aConfig, aConfig.mBaseSeed + i);

        renderers[i]->mMaxPathLength = aConfig.mMaxPathLength;
        renderers[i]->mMinPathLength = aConfig.mMinPathLength;
    }

    clock_t startT = clock();
    int iter = 0;

    // Rendering loop, when we have any time limit, use time-based loop,
    // otherwise go with required iterations
    if(aConfig.mMaxTime > 0)
    {
        // Time based loop
#pragma omp parallel
        while(clock() < startT + aConfig.mMaxTime*CLOCKS_PER_SEC)
        {
            int threadId = omp_get_thread_num();
            renderers[threadId]->RunIteration(iter);

#pragma omp atomic
            iter++; // counts number of iterations
        }
    }
    else
    {
        // Iterations based loop
#pragma omp parallel for
        for(iter=0; iter < aConfig.mIterations; iter++)
        {
            int threadId = omp_get_thread_num();
            renderers[threadId]->RunIteration(iter);
        }
    }

    clock_t endT = clock();

    if(oUsedIterations)
        *oUsedIterations = iter+1;

    // Accumulate from all renderers into a common framebuffer
    int usedRenderers = 0;

    // With very low number of iterations and high number of threads
    // not all created renderers had to have been used.
    // Those must not participate in accumulation.
    for(int i=0; i<aConfig.mNumThreads; i++)
    {
        if(!renderers[i]->WasUsed())
            continue;

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

    // Scale framebuffer by the number of used renderers
    aConfig.mFramebuffer->Scale(1.f / usedRenderers);

    // Clean up renderers
    for(int i=0; i<aConfig.mNumThreads; i++)
        delete renderers[i];

    delete [] renderers;

    return float(endT - startT) / CLOCKS_PER_SEC;
}

//////////////////////////////////////////////////////////////////////////
// Generates index.html with all scene-algorithm combinations.

void FullReport(const Config &aConfig)
{
    // Make a local copy of config
    Config config = aConfig;

    config.mFullReport = false;

    // Setup framebuffer and threads
    Framebuffer fbuffer;
    config.mFramebuffer = &fbuffer;

    // Setup html writer
    HtmlWriter html_writer("index.html");
    html_writer.WriteHeader();
    html_writer.mAlgorithmCount = (int)Config::kAlgorithmMax;
    html_writer.mThumbnailSize  = 128;

    int numIterations;

    if(SizeOfArray(g_SceneConfigs) != 4)
    {
        printf("Report assumes we have only 4 scenes\n");
        printf("Cannot continue with %d\n", SizeOfArray(g_SceneConfigs));
        exit(2);
    }

    // Quite subjective evaluation whether given algorithm is
    // for a given scene good, poor, or neutral.
    std::set<int> goodAlgorithms[4], poorAlgorithms[4];
    goodAlgorithms[0].insert(Config::kVertexConnectionMerging);
    goodAlgorithms[0].insert(Config::kBidirectionalPhotonMapping);
    poorAlgorithms[0].insert(Config::kBidirectionalPathTracing);

    goodAlgorithms[1].insert(Config::kVertexConnectionMerging);
    goodAlgorithms[1].insert(Config::kBidirectionalPhotonMapping);
    poorAlgorithms[1].insert(Config::kBidirectionalPathTracing);
    poorAlgorithms[1].insert(Config::kProgressivePhotonMapping);

    goodAlgorithms[2].insert(Config::kVertexConnectionMerging);
    goodAlgorithms[2].insert(Config::kBidirectionalPhotonMapping);
    poorAlgorithms[2].insert(Config::kProgressivePhotonMapping);

    goodAlgorithms[3].insert(Config::kVertexConnectionMerging);
    goodAlgorithms[3].insert(Config::kBidirectionalPathTracing);
    poorAlgorithms[3].insert(Config::kBidirectionalPhotonMapping);
    poorAlgorithms[3].insert(Config::kProgressivePhotonMapping);

    // Acronyms of algorithms in four-way split
    std::string splitAcronyms[] = {"PPM", "BPM", "BPT", "VCM"};
    // Filename and border color for images in four-way split
    std::string splitFiles[4];
    int         borderColors[4];

    clock_t startTime = clock();

    for(int sceneID=0; sceneID<SizeOfArray(g_SceneConfigs); sceneID++)
    {
        Scene  scene;
        scene.LoadCornellBox(config.mResolution, g_SceneConfigs[sceneID]);
        scene.BuildSceneSphere();
        config.mScene = &scene;

        html_writer.AddScene(scene.mSceneName);
        printf("Scene: %s\n", scene.mSceneName.c_str());

        for(uint algID = 0; algID < (uint)Config::kAlgorithmMax; algID++)
        {
            config.mAlgorithm = Config::Algorithm(algID);
            printf("Running %s... ", config.GetName(config.mAlgorithm));
            fflush(stdout);
            float time = render(config, &numIterations);
            printf("done in %.2f s\n", time);

            std::string filename = DefaultFilename(g_SceneConfigs[sceneID],
                *config.mScene, config.mAlgorithm);

            fbuffer.SaveBMP(filename.c_str(), 2.2f);

            // Add thumbnail of the method
            HtmlWriter::BorderColor bcolor = HtmlWriter::kNone;

            if(poorAlgorithms[sceneID].count(algID) > 0)
                bcolor = HtmlWriter::kRed;

            if(goodAlgorithms[sceneID].count(algID) > 0)
                bcolor = HtmlWriter::kGreen;

            html_writer.AddRendering(Config::GetName(config.mAlgorithm),
                filename, time, bcolor,
                html_writer.MakeMessage("<br/>Iterations: %d", numIterations));

            if(algID >= (int)Config::kProgressivePhotonMapping)
            {
                const int idx = algID - Config::kProgressivePhotonMapping;
                splitFiles[idx]   = filename;
                borderColors[idx] = bcolor;
            }
        }

        html_writer.AddFourWaySplit(splitFiles, splitAcronyms,
            borderColors, config.mResolution.x);
    }

    html_writer.Close();

    clock_t endTime = clock();
    printf("Whole run took %.2f s\n", float(endTime - startTime) / CLOCKS_PER_SEC);
}

//////////////////////////////////////////////////////////////////////////
// Main

int main(int argc, const char *argv[])
{
    // Warns when not using C++11 Mersenne Twister
    PrintRngWarning();

    // Setups config based on command line
    Config config;
    ParseCommandline(argc, argv, config);

    // If number of threads is invalid, set 1 thread per processor
    if(config.mNumThreads <= 0)
        config.mNumThreads  = std::max(1, omp_get_num_procs());

    if(config.mFullReport)
    {
        FullReport(config);
        return 0;
    }

    // When some error has been encountered, exits
    if(config.mScene == NULL)
        return 1;

    // Sets up framebuffer and number of threads
    Framebuffer fbuffer;
    config.mFramebuffer = &fbuffer;

    // Prints what we are doing
    printf("Scene:   %s\n", config.mScene->mSceneName.c_str());
    if(config.mMaxTime > 0)
        printf("Target:  %f seconds render time\n", config.mMaxTime);
    else
        printf("Target:  %d iteration(s)\n", config.mIterations);

    // Renders the image
    printf("Running: %s... ", config.GetName(config.mAlgorithm));
    fflush(stdout);
    float time = render(config);
    printf("done in %.2f s\n", time);

    // Saves the image
    std::string extension = config.mOutputName.substr(config.mOutputName.length() - 3, 3);

    if(extension == "bmp")
        fbuffer.SaveBMP(config.mOutputName.c_str(), 2.2f /*gamma*/);
    else if(extension == "hdr")
        fbuffer.SaveHDR(config.mOutputName.c_str());
    else
        printf("Used unknown extension %s\n", extension.c_str());

    // Scene cleanup
    delete config.mScene;

    return 0;
}
