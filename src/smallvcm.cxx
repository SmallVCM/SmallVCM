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
#include <sstream>

// Renderer configuration, holds algorithm, scene, and all other settings
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

    static const char* GetName(Algorithm aAlgorithm)
    {
        static const char* algorithmNames[7] =
        {
            "eye light",
            "path tracing",
            "light tracing",
            "progressive photon mapping",
            "bidirectional photon mapping",
            "bidirectional path tracing",
            "vertex connection and merging"
        };

        if(aAlgorithm < 0 || aAlgorithm > 7)
            return "unknown algorithm";

        return algorithmNames[aAlgorithm];
    }

    static const char* GetAcronym(Algorithm aAlgorithm)
    {
        static const char* algorithmNames[7] = {
            "el", "pt", "lt", "ppm", "bpm", "bpt", "vcm" };

        if(aAlgorithm < 0 || aAlgorithm > 7)
            return "unknown";
        return algorithmNames[aAlgorithm];
    }

    const Scene *mScene;
    Algorithm   mAlgorithm;
    int         mIterations;
    float       mMaxTime;
    Framebuffer *mFramebuffer;
    int         mNumThreads;
    int         mBaseSeed;
    uint        mMaxPathLength;
    uint        mMinPathLength;
    std::string mOutputName;
    Vec2i       mResolution;
    bool        mFullReport; // ignore scene and algorithm and do html report instead
};

// Utility function, essentially a renderer factory
AbstractRenderer* CreateRenderer(
    const Config::Algorithm  aAlgorithm,
    const Scene              &aScene,
    const int                aSeed)
{
    switch(aAlgorithm)
    {
    case Config::kEyeLight:
        return new EyeLight(aScene, aSeed);
    case Config::kPathTracing:
        return new PathTracer(aScene, aSeed);
    case Config::kLightTracing:
        return new VertexCM(aScene, VertexCM::kLightTrace, aSeed);
    case Config::kProgressivePhotonMapping:
        return new VertexCM(aScene, VertexCM::kPpm, aSeed);
    case Config::kBidirectionalPhotonMapping:
        return new VertexCM(aScene, VertexCM::kBpm, aSeed);
    case Config::kBidirectionalPathTracing:
        return new VertexCM(aScene,VertexCM::kBpt, aSeed);
    case Config::kVertexConnectionMerging:
        return new VertexCM(aScene, VertexCM::kVcm, aSeed);
    default:
        printf("Unknown algorithm!!\n");
        exit(2);
    }
}

// Scene configurations
uint g_SceneConfigs[] = {
    Scene::kGlossyFloor | Scene::kBothSmallSpheres  | Scene::kLightSun,
    Scene::kGlossyFloor | Scene::kLargeMirrorSphere | Scene::kLightCeiling,
    Scene::kGlossyFloor | Scene::kBothSmallSpheres  | Scene::kLightPoint,
    Scene::kGlossyFloor | Scene::kBothSmallSpheres  | Scene::kLightBackground
};

std::string DefaultFilename(
    const uint              aSceneConfig,
    const Scene             &aScene,
    const Config::Algorithm aAlgorithm)
{
    std::string filename;
    // if scene has glossy floor, name will be prefixed by g
    if((aSceneConfig & Scene::kGlossyFloor) != 0)
        filename = "g";

    // We use scene acronym
    filename += aScene.mSceneAcronym;

    // We add acronym of the used algorithm
    filename += "_";
    filename += Config::GetAcronym(aAlgorithm);

    // And it will be written as bmp
    filename += ".bmp";

    return filename;
}

// Utility function, gives length of array
template <typename T, size_t N>
inline int SizeOfArray( const T(&)[ N ] )
{
    return int(N);
}

void PrintRngWarning()
{
#if defined(LEGACY_RNG)
    printf("The code was not compiled for C++11.\n");
    printf("It will be using Tiny Encryption Algorithm-based"
        "random number generator.\n");
    printf("This is worse than the Mersenne Twister from C++11.\n");
    printf("Consider setting up for C++11.\n");
    printf("Visual Studio 2010, and g++ 4.6.3 and later work.\n\n");
#endif
}

void PrintHelp(const char *argv[])
{
    printf("\n");
    printf("Usage: %s [ -s <scene_id> | -a <algorithm>\n", argv[0]);
    printf("           -t <time> | -i <iteration> | -o <output_name>\n");
    printf("           --report ]\n\n");
    printf("    -s  Selects the scene (default 0):\n");

    for(int i = 0; i < SizeOfArray(g_SceneConfigs); i++)
        printf("          %d    %s\n", i, Scene::GetSceneName(g_SceneConfigs[i]).c_str());

    printf("    -a  Selects the rendering algorithm (default vcm):\n");

    for(int i = 0; i < (int)Config::kAlgorithmMax; i++)
        printf("          %-3s  %s\n",
            Config::GetAcronym(Config::Algorithm(i)),
            Config::GetName(Config::Algorithm(i)));

    printf("    -t  Number of seconds to run the algorithm\n");
    printf("    -i  Number of iterations to run the algorithm (default 1)\n");
    printf("    -o  User specified output name, with extension .bmp or .hdr (default .bmp)\n");
    printf("    --report  Generates index.html presenting all scene-algorithm combinations\n");
    printf("              Obeys -t and -i options, ignores the rest.\n");
    printf("              Recommended usage: --report -t 10  (takes 5.5 mins)\n");
    printf("              Recommended usage: --report -t 60  (takes 60 mins)\n");
    printf("\n    Note: Time (-t) takes precedence over iterations (-i) if both are defined\n");
}

// Parses command line, setting up config
void ParseCommandline(int argc, const char *argv[], Config &oConfig)
{
    // Parameters marked with [cmd] can be change from command line
    oConfig.mScene         = NULL;                  // [cmd] When NULL, renderer will not run
    oConfig.mAlgorithm     = Config::kAlgorithmMax; // [cmd]
    oConfig.mIterations    = 1;                     // [cmd]
    oConfig.mMaxTime       = -1.f;                  // [cmd]
    //oConfig.mFramebuffer   = NULL; // this is never set by any parameter
    oConfig.mNumThreads    = 1;
    oConfig.mBaseSeed      = 1234;
    oConfig.mMaxPathLength = 10;
    oConfig.mMinPathLength = 0;
    oConfig.mOutputName    = "";                    // [cmd]
    oConfig.mResolution    = Vec2i(512, 512);
    oConfig.mFullReport    = false;

    int sceneID    = 0; // default 0

    // Load arguments
    for(int i=1; i<argc; i++)
    {
        std::string arg(argv[i]);

        // print help string (at any position)
        if(arg == "-h" || arg == "--help" || arg == "/?")
        {
            PrintHelp(argv);
            return;
        }

        if(arg[0] != '-') // all our commands start with -
        {
            continue;
        }
        else if(arg == "--report")
        {
            oConfig.mFullReport = true;
        }
        else if(arg == "-s") // scene to load
        {
            if(++i == argc)
            {
                printf("Missing <sceneID> argument, please see help (-h)\n");
                return;
            }

            std::istringstream iss(argv[i]);
            iss >> sceneID;

            if(iss.fail() || sceneID < 0 || sceneID >= SizeOfArray(g_SceneConfigs))
            {
                printf("Invalid <sceneID> argument, please see help (-h)\n");
                return;
            }
        }
        else if(arg == "-a") // algorithm to use
        {
            if(++i == argc)
            {
                printf("Missing <algorithm> argument, please see help (-h)\n");
                return;
            }

            std::string alg(argv[i]);
            for(int i=0; i<Config::kAlgorithmMax; i++)
                if(alg == Config::GetAcronym(Config::Algorithm(i)))
                    oConfig.mAlgorithm = Config::Algorithm(i);

            if(oConfig.mAlgorithm == Config::kAlgorithmMax)
            {
                printf("Invalid <algorithm> argument, please see help (-h)\n");
                return;
            }
        }
        else if(arg == "-i") // number of iterations to run
        {
            if(++i == argc)
            {
                printf("Missing <iteration> argument, please see help (-h)\n");
                return;
            }

            std::istringstream iss(argv[i]);
            iss >> oConfig.mIterations;

            if(iss.fail() || oConfig.mIterations < 1)
            {
                printf("Invalid <iteration> argument, please see help (-h)\n");
                return;
            }
        }
        else if(arg == "-t") // number of seconds to run
        {
            if(++i == argc)
            {
                printf("Missing <time> argument, please see help (-h)\n");
                return;
            }

            std::istringstream iss(argv[i]);
            iss >> oConfig.mMaxTime;

            if(iss.fail() || oConfig.mMaxTime < 0)
            {
                printf("Invalid <time> argument, please see help (-h)\n");
                return;
            }

            oConfig.mIterations = -1; // time has precedence
        }
        else if(arg == "-o") // number of seconds to run
        {
            if(++i == argc)
            {
                printf("Missing <output_name> argument, please see help (-h)\n");
                return;
            }

            oConfig.mOutputName = argv[i];

            if(oConfig.mOutputName.length() == 0)
            {
                printf("Invalid <output_name> argument, please see help (-h)\n");
                return;
            }
        }
    }

    // When doing full report, we ignore algorithm and scene settings
    if(oConfig.mFullReport)
        return;

    // Check algorithm was selected
    if(oConfig.mAlgorithm == Config::kAlgorithmMax)
    {
        oConfig.mAlgorithm = Config::kVertexConnectionMerging;
    }

    // Load scene
    Scene *scene = new Scene;
    scene->LoadCornellBox(oConfig.mResolution, g_SceneConfigs[sceneID]);
    scene->BuildSceneSphere();

    oConfig.mScene = scene;

    // If no output name is chosen, create a default one
    if(oConfig.mOutputName.length() == 0)
    {
        oConfig.mOutputName = DefaultFilename(g_SceneConfigs[sceneID],
            *oConfig.mScene, oConfig.mAlgorithm);
    }

    // Check if output name has valid extension (.bmp or .hdr) and if not add .bmp
    std::string extension = "";

    if(oConfig.mOutputName.length() > 4) // must be at least 1 character before .bmp
        extension = oConfig.mOutputName.substr(
            oConfig.mOutputName.length() - 4, 4);

    if(extension != ".bmp" && extension != ".hdr")
        oConfig.mOutputName += ".bmp";
}

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
        renderers[i] = CreateRenderer(aConfig.mAlgorithm,
            *aConfig.mScene, aConfig.mBaseSeed + i);

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
    config.mNumThreads  = std::max(1, omp_get_num_procs());

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
    config.mNumThreads  = std::max(1, omp_get_num_procs());

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
    else if(extension == "bmp")
        fbuffer.SaveHDR(config.mOutputName.c_str());

    // Scene cleanup
    delete config.mScene;

    return 0;
}
