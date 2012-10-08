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

#include <omp.h>
#include <string>
#include <set>

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
        kVertexConnectionMerging,
        kAlgorithmMax
    };

    const char* GetName()
    {
        static const char* algorithmNames[7] =
        {
            "Eye Light",
            "Path Tracing",
            "Light Tracing",
            "Progressive Photon Mapping",
            "Bidirectional Photon Mapping",
            "Bidirectional Path Tracing",
            "Vertex Connection Merging"
        };

        if(mAlgorithm < 0 || mAlgorithm > 7)
            return "Unknown algorithm";

        return algorithmNames[mAlgorithm];
    }

    const char* GetAcronym()
    {
        static const char* algorithmNames[7] = {
            "el", "pt", "lt", "ppm", "bpm", "bpt", "vcm" };

        if(mAlgorithm < 0 || mAlgorithm > 7)
            return "unknown";
        return algorithmNames[mAlgorithm];
    }

    const Scene *mScene;
    Algorithm   mAlgorithm;
    int         mIterations;
    float       mMaxTime;
    bool        mUseMaxTime; // otherwise use iterations
    Framebuffer *mFramebuffer;
    int         mNumThreads;
    int         mBaseSeed;
    uint        mMaxPathLength;
    uint        mMinPathLength;
};


float render(const Config &aConfig, int *oUsedIterations = NULL)
{
    omp_set_num_threads(aConfig.mNumThreads);

    typedef AbstractRenderer* AbstractRendererPtr;
    AbstractRendererPtr *renderers;
    renderers = new AbstractRendererPtr[aConfig.mNumThreads];

    int iterations = aConfig.mIterations;
    bool use_time  = aConfig.mUseMaxTime;

    switch(aConfig.mAlgorithm)
    {
    case Config::kEyeLight:
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new EyeLight(*aConfig.mScene);
        // iterations have no meaning for kEyeLight
        iterations = 1;
        use_time   = false;
        break;
    case Config::kPathTracing:
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new PathTracer(*aConfig.mScene, aConfig.mBaseSeed + i);
        break;
    case Config::kLightTracing:
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(
                *aConfig.mScene, VertexCM::kLightTrace, aConfig.mBaseSeed + i);
        break;
    case Config::kProgressivePhotonMapping:
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(
                *aConfig.mScene, VertexCM::kPpm, aConfig.mBaseSeed + i);
        break;
    case Config::kBidirectionalPhotonMapping:
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(
                *aConfig.mScene, VertexCM::kBpm, aConfig.mBaseSeed + i);
        break;
    case Config::kBidirectionalPathTracing:
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(
                *aConfig.mScene,VertexCM::kBpt, aConfig.mBaseSeed + i);
        break;
    case Config::kVertexConnectionMerging:
        for(int i=0; i<aConfig.mNumThreads; i++)
            renderers[i] = new VertexCM(
                *aConfig.mScene, VertexCM::kVcm, aConfig.mBaseSeed + i);
        break;
    }

    for(int i=0; i<aConfig.mNumThreads; i++)
    {
        renderers[i]->mMaxPathLength = aConfig.mMaxPathLength;
        renderers[i]->mMinPathLength = aConfig.mMinPathLength;
    }

    clock_t startT = clock();
    int iter = 0;

    if(use_time)
    {
        #pragma omp parallel
        while(clock() < startT + aConfig.mMaxTime*CLOCKS_PER_SEC)
        {
            int threadId = omp_get_thread_num();
            renderers[threadId]->RunIteration(iter);
            
            #pragma omp atomic
            iter++;
        }
    }
    else
    {
        #pragma omp parallel for
        for(iter=0; iter < iterations; iter++)
        {
            int threadId = omp_get_thread_num();
            renderers[threadId]->RunIteration(iter);
        }
    }

    clock_t endT = clock();

    if(oUsedIterations)
        *oUsedIterations = iter+1;

    int usedRenderers = 0;

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

    aConfig.mFramebuffer->Scale(1.f / usedRenderers);

    for(int i=0; i<aConfig.mNumThreads; i++)
        delete renderers[i];

    delete [] renderers;

    return float(endT - startT) / CLOCKS_PER_SEC;
}

struct SceneConfig
{
    SceneConfig()
    {}

    SceneConfig(uint aMask) :
        mMask(aMask)
    {}

    uint mMask;
    std::set<int> mGoodAlgorithm;
    std::set<int> mPoorAlgorithm;
};

int main(int argc, const char *argv[])
{
    int   base_iterations = 1;
    Vec2i resolution      = Vec2i(512, 512);
    int   max_path_length = 10;
    int   min_path_length = 0;
    float max_time        = 60;
    bool  use_max_time    = false;

    if(argc > 1)
        base_iterations = atoi(argv[1]);

    if(argc > 2)
    {
        max_path_length = atoi(argv[2]);
        min_path_length = atoi(argv[2]);
    }

#if defined(LEGACY_RNG)
    printf("The code was not compiled for C++11.\n");
    printf("It will be using Tiny Encryption Algorithm-based"
        "random number generator.\n");
    printf("This is worse than the Mersenne Twister from C++11.\n");
    printf("Consider setting up for C++11.\n");
    printf("Visual Studio 2010, and g++ 4.6.3 and later work.\n\n");
#endif

    const int numThreads = std::max(1, omp_get_num_procs());
    printf("Using %d threads\n", numThreads);

    SceneConfig sceneConfigs[] = {
        SceneConfig(Scene::kGlossyFloor | Scene::kBothSmallSpheres  | Scene::kLightSun),
        SceneConfig(Scene::kGlossyFloor | Scene::kLargeMirrorSphere | Scene::kLightCeiling),
        SceneConfig(Scene::kGlossyFloor | Scene::kBothSmallSpheres  | Scene::kLightPoint),
        SceneConfig(Scene::kGlossyFloor | Scene::kBothSmallSpheres  | Scene::kLightBackground)
    };

    // Set (subjective) good/poor/neutral algorithms, to get colors for report
    sceneConfigs[0].mGoodAlgorithm.insert(Config::kVertexConnectionMerging);
    sceneConfigs[0].mGoodAlgorithm.insert(Config::kBidirectionalPhotonMapping);
    sceneConfigs[0].mPoorAlgorithm.insert(Config::kBidirectionalPathTracing);

    sceneConfigs[1].mGoodAlgorithm.insert(Config::kVertexConnectionMerging);
    sceneConfigs[1].mGoodAlgorithm.insert(Config::kBidirectionalPhotonMapping);
    sceneConfigs[1].mPoorAlgorithm.insert(Config::kBidirectionalPathTracing);
    sceneConfigs[1].mPoorAlgorithm.insert(Config::kProgressivePhotonMapping);

    sceneConfigs[2].mGoodAlgorithm.insert(Config::kVertexConnectionMerging);
    sceneConfigs[2].mGoodAlgorithm.insert(Config::kBidirectionalPhotonMapping);
    sceneConfigs[2].mPoorAlgorithm.insert(Config::kProgressivePhotonMapping);

    sceneConfigs[3].mGoodAlgorithm.insert(Config::kVertexConnectionMerging);
    sceneConfigs[3].mGoodAlgorithm.insert(Config::kBidirectionalPathTracing);
    sceneConfigs[3].mPoorAlgorithm.insert(Config::kBidirectionalPhotonMapping);
    sceneConfigs[3].mPoorAlgorithm.insert(Config::kProgressivePhotonMapping);


    const int sceneConfigCount = sizeof(sceneConfigs) / sizeof(SceneConfig);

    Framebuffer fbuffer;
    Config      config;
    config.mIterations    = base_iterations;
    config.mMaxTime       = max_time;
    config.mUseMaxTime    = use_max_time;
    config.mFramebuffer   = &fbuffer;
    config.mNumThreads    = numThreads;
    config.mBaseSeed      = 1234;
    config.mMaxPathLength = max_path_length;
    config.mMinPathLength = min_path_length;

    HtmlWriter html_writer("report/index.html");
    html_writer.WriteHeader();
    int thumbnailSize = 128;

    int algorithmMask[7] =
    {
        1, // kEyeLight
        1, // kPathTracing
        1, // kLightTracing
        1, // kProgressivePhotonMapping
        1, // kBidirectionalPhotonMapping
        1, // kBidirectionalPathTracing
        1  // kVertexConnectionMerging
    };

    std::string FourWaySplitFiles[4];
    std::string FourWaySplitNames[4];
    int         BorderColors[4];
    uint        FourWaySplitAlgorithms[4] =
    {
        Config::kProgressivePhotonMapping,
        Config::kBidirectionalPhotonMapping,
        Config::kBidirectionalPathTracing,
        Config::kVertexConnectionMerging,
    };

    for(int i=0; i<4; i++)
    {
        config.mAlgorithm = Config::Algorithm(FourWaySplitAlgorithms[i]);
        
        std::string acronym = config.GetAcronym();

        for(uint j=0; j<acronym.length(); j++)
            if(acronym[j] >= 'a' && acronym[j] <= 'z')
                acronym[j] += 'A' - 'a';

        FourWaySplitNames[i] = acronym;
    }

    int algorithmCount = 0;

    for(uint algId = 0; algId < Config::kAlgorithmMax; algId++)
        if(algorithmMask[algId])
            algorithmCount++;

    html_writer.mAlgorithmCount = algorithmCount;

    clock_t startTime = clock();

    for(int sceneId = 0; sceneId < sceneConfigCount; sceneId++)
    {
        uint mask = sceneConfigs[sceneId].mMask;

        Scene scene;
        scene.LoadCornellBox(resolution, mask);
        scene.BuildSceneSphere();
        config.mScene = &scene;

        std::string name, acronym;
        name = scene.GetSceneName(mask, &acronym);

        std::string sceneFilename(acronym);
        if((mask & Scene::kGlossyFloor) != 0)
            sceneFilename = "g" + sceneFilename;

        html_writer.AddScene(name);
        printf("Scene: %s\n", name.c_str());

        int numIterations;

        for(uint algId = 0; algId < Config::kAlgorithmMax; algId++)
        {
            if(!algorithmMask[algId]) continue;
            config.mAlgorithm = Config::Algorithm(algId);
            printf("Running %s... ", config.GetName());
            fflush(stdout);
            float time = render(config, &numIterations);
            printf("done in %.2f s\n", time);
            config.mBaseSeed += config.mNumThreads;

            std::string filename = sceneFilename + "_" + config.GetAcronym() + ".bmp";

            // HTML output
            fbuffer.SaveBMP(("report/" + filename).c_str(), 2.2f);
            HtmlWriter::BorderColor bcolor = HtmlWriter::kNone;

            if(sceneConfigs[sceneId].mGoodAlgorithm.count(algId) > 0)
                bcolor = HtmlWriter::kGreen;
            
            if(sceneConfigs[sceneId].mPoorAlgorithm.count(algId) > 0)
                bcolor = HtmlWriter::kRed;

            html_writer.AddRendering(config.GetName(), filename, time, bcolor,
                html_writer.MakeMessage("<br/>Iterations: %d", numIterations));

            for(int i=0; i<4; i++)
            {
                if(algId == FourWaySplitAlgorithms[i])
                    FourWaySplitFiles[i] = filename;
            }
        }

        for(int i=0; i<4; i++)
        {
            BorderColors[i] = HtmlWriter::kNone;

            if(sceneConfigs[sceneId].mGoodAlgorithm.count(
                FourWaySplitAlgorithms[i]) > 0)
                BorderColors[i] = HtmlWriter::kGreen;

            if(sceneConfigs[sceneId].mPoorAlgorithm.count(
                FourWaySplitAlgorithms[i]) > 0)
                BorderColors[i] = HtmlWriter::kRed;
        }

        html_writer.AddFourWaySplit(FourWaySplitFiles,
            FourWaySplitNames, BorderColors, resolution.x);
    }

    html_writer.Close();
    
    clock_t endTime = clock();
    printf("Whole run took %.2f s\n", float(endTime - startTime) / CLOCKS_PER_SEC);
}
