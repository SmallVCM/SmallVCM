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

#ifndef __VERTEXCM_HXX__
#define __VERTEXCM_HXX__

#include <vector>
#include <cmath>
#include "renderer.hxx"
#include "bsdf.hxx"
#include "rng.hxx"
#include "hashgrid.hxx"

class VertexCM : public AbstractRenderer
{
    // The sole point of this structure is to make carrying around the ray baggage easier.
    struct PathElement
    {
        Vec3f mOrigin;             // Path origin
        Vec3f mDirection;          // Where to go next
        Vec3f mThroughput;         // Path throughput (multiplied by camera importance)
        uint  mPathLength    : 30; // Number of path segments, including this
        uint  mIsFiniteLight :  1; // Just generate by finite light
        uint  mSpecularPath  :  1; // All bounces so far were specular

        // We compute MIS in a cumulative fashion. 1 variable is used,
        // plus 1 for each used method (connection, merging).
        // Please see the VCM implementation tech report for derivation.

        float d0;   // Common helper variable for MIS
        float d1vc; // Helper variable for vertex connection MIS
        float d1vm; // Helper variable for vertex merging MIS
    };

    // Path vertex, used for merging and connection
    template<bool tFromLight>
    struct PathVertex
    {
        Vec3f mHitpoint;   // Position of the vertex
        Vec3f mThroughput; // Path throughput (multiplied by emission)
        uint  mPathLength; // Number of segments between source and vertex

        // Stores all required local information, including incoming direction.
        BSDF<tFromLight> mBsdf;

        // We compute MIS in a cumulative fashion. 1 variable is used,
        // plus 1 for each used method (connection, merging).
        // Please see the accompanying writeup for derivation.

        float d0;   // Common helper variable for MIS
        float d1vc; // Helper variable for vertex connection MIS
        float d1vm; // Helper variable for vertex merging MIS

        // Used by HashGrid
        const Vec3f& GetPosition() const
        {
            return mHitpoint;
        }
    };

    typedef PathVertex<false> CameraVertex;
    typedef PathVertex<true>  LightVertex;

    typedef BSDF<false>       CameraBSDF;
    typedef BSDF<true>        LightBSDF;

    // Range query used for PPM, BPT, and VCM. When HashGrid finds a vertex
    // within range -- Process() is called and vertex
    // merging is performed. BSDF of the camera vertex is used.
    class RangeQuery
    {
    public:

        RangeQuery(
            const VertexCM    &aVertexCM,
            const Vec3f       &aCameraPosition,
            const CameraBSDF  &aCameraBsdf,
            const PathElement &aCameraSample
        ) : 
            mVertexCM(aVertexCM),
            mCameraPosition(aCameraPosition),
            mCameraBsdf(aCameraBsdf),
            mCameraSample(aCameraSample),
            mContrib(0)
        {}

        const Vec3f& GetPosition() const { return mCameraPosition; }

        const Vec3f& GetContrib() const { return mContrib; }

        void Process(const LightVertex& aLightVertex)
        {
            if((aLightVertex.mPathLength + mCameraSample.mPathLength > mVertexCM.mMaxPathLength) ||
               (aLightVertex.mPathLength + mCameraSample.mPathLength < mVertexCM.mMinPathLength))
                 return;

            // Retrieve light incoming direction in world coordinates
            const Vec3f lightDirection = aLightVertex.mBsdf.WorldDirFix();

            float cosCamera, cameraBsdfDirPdfW, cameraBsdfRevPdfW;
            const Vec3f cameraBsdfFactor = mCameraBsdf.Evaluate(
                mVertexCM.mScene, lightDirection, cosCamera, &cameraBsdfDirPdfW,
                &cameraBsdfRevPdfW);

            if(cameraBsdfFactor.IsZero())
                return;

            cameraBsdfDirPdfW *= mCameraBsdf.ContinuationProb();

            // Even though this is pdf from camera BSDF, the continuation probability
            // must come from light BSDF, because that would govern it if light path
            // actually continued
            cameraBsdfRevPdfW *= aLightVertex.mBsdf.ContinuationProb();

            const float wLight = aLightVertex.d0 * mVertexCM.mMisVcWeightFactor +
                aLightVertex.d1vm * mVertexCM.Mis(cameraBsdfDirPdfW);
            const float wCamera = mCameraSample.d0 * mVertexCM.mMisVcWeightFactor +
                mCameraSample.d1vm * mVertexCM.Mis(cameraBsdfRevPdfW);

            // Ppm merges, but does not have MIS weights
            const float misWeight = mVertexCM.mPpm ?
                1.f :
                1.f / (wLight + 1.f + wCamera);

            mContrib += misWeight * cameraBsdfFactor * aLightVertex.mThroughput;
        }

    private:

        const VertexCM    &mVertexCM;
        const Vec3f       &mCameraPosition;
        const CameraBSDF  &mCameraBsdf;
        const PathElement &mCameraSample;
        Vec3f             mContrib;
    };

public:

    enum AlgorithmType
    {
        // light vertices contribute to camera,
        // No MIS weights (d0, d1vm, d1vc all ignored)
        kLightTrace = 0,
        // Camera and light vertices merged on first non-specular camera bounce.
        // Cannot handle mixed specular + non-specular materials.
        // No MIS weights (d0, d1vm, d1vc all ignored)
        kPpm,
        // Camera and light vertices merged on along full path.
        // d0 and d1vm used for MIS
        kBpm,
        // Standard bidirectional path tracing
        // d0 and d1vc used for MIS
        kBpt,
        // Vertex connection and mering
        // d0, d1vm, and d1vc used for MIS
        kVcm
    };

public:

    VertexCM(
        const Scene&  aScene,
        AlgorithmType aAlgorithm,
        int           aSeed = 1234
    ) :
        AbstractRenderer(aScene),
        mRng(aSeed),
        mLightTraceOnly(false),
        mUseVC(false),
        mUseVM(false),
        mPpm(false)
    {
        switch(aAlgorithm)
        {
        case kLightTrace:
            mLightTraceOnly = true;
            break;
        case kPpm:
            mPpm   = true;
            mUseVM = true;
            break;
        case kBpm:
            mUseVM = true;
            break;
        case kBpt:
            mUseVC = true;
            break;
        case kVcm:
            mUseVC = true;
            mUseVM = true;
            break;
        default:
            printf("Unknown algorithm requested\n");
            break;
        }

        if(mPpm)
        {
            // We will check the scene to make sure it does not contain mixed
            // specular and non-specular materials
            for(int i = 0; i < mScene.GetMaterialCount(); ++i)
            {
                const Material &mat = mScene.GetMaterial(i);

                const bool hasNonSpecular =
                    (mat.mDiffuseReflectance.Max() > 0) ||
                    (mat.mPhongReflectance.Max() > 0);

                const bool hasSpecular =
                    (mat.mMirrorReflectance.Max() > 0) ||
                    (mat.mIOR > 0);

                if(hasNonSpecular && hasSpecular)
                {
                    printf(
                        "*WARNING* Our Ppm implementation cannot handle materials mixing\n"
                        "Specular and NonSpecular BSDFs. The extension would be\n"
                        "fairly straightforward. In BounceSample for CameraSample\n"
                        "limit the considered events to Specular only.\n"
                        "Merging will use non-specular components, bounce will be specular.\n"
                        "If there is no specular component, the ray will terminate.\n\n");

                    printf("We are now switching from *Ppm* to *Bpm*, which can handle the scene\n\n");

                    mPpm = false;
                }
            }
        }

        mBaseRadius  = 0.002f * mScene.mSceneSphere.mSceneRadius;
        mRadiusAlpha = 0.75f;
    }

    virtual void RunIteration(int aIteration)
    {
        // While we have the same number of pixels (camera paths)
        // and light paths, we do keep them separate for clarity reasons
        const int resX = int(mScene.mCamera.mResolution.x);
        const int resY = int(mScene.mCamera.mResolution.y);
        const int pathCount = resX * resY;
        mScreenPixelCount = float(resX * resY);
        mLightPathCount   = float(resX * resY);

        // Setup our radius, 1st iteration has aIteration == 0, thus offset
        float radius = mBaseRadius;
        radius /= std::pow(float(aIteration + 1), 0.5f * (1 - mRadiusAlpha));
        // Purely for numeric stability
        radius = std::max(radius, 1e-7f);
        const float radiusSqr = Sqr(radius);

        // Factor used to normalise vertex merging contribution.
        // We divide the summed up energy by disk radius and number of light paths
        mVmNormalization = 1.f / (radiusSqr * PI_F * mLightPathCount);

        // Set up VC and VM MIS weight factors
        const float baseVmWeightFactor = (PI_F * radiusSqr) * pathCount;
        mMisVmWeightFactor = mUseVM ? Mis(baseVmWeightFactor)       : 0.f;
        mMisVcWeightFactor = mUseVC ? Mis(1.f / baseVmWeightFactor) : 0.f;

        // Clear path ends, nothing ends anywhere
        mPathEnds.resize(pathCount);
        memset(&mPathEnds[0], 0, mPathEnds.size() * sizeof(int));

        // Remove all light vertices and reserve space for some
        mLightVertices.reserve(pathCount);
        mLightVertices.clear();

        //////////////////////////////////////////////////////////////////////////
        // Generate light paths
        //////////////////////////////////////////////////////////////////////////
        for(int pathIdx = 0; pathIdx < pathCount; pathIdx++)
        {
            PathElement lightSample;
            GenerateLightSample(lightSample);

            //////////////////////////////////////////////////////////////////////////
            // Trace light path
            for(;; ++lightSample.mPathLength)
            {
                // We offset ray origin instead of setting tmin due to numeric
                // issues in ray-sphere intersection. The isect.dist has to be
                // extended by this EPS_RAY after hit point is determined
                Ray ray(lightSample.mOrigin + lightSample.mDirection * EPS_RAY,
                    lightSample.mDirection, 0);
                Isect isect(1e36f);

                if(!mScene.Intersect(ray, isect))
                    break;

                const Vec3f hitPoint = ray.org + ray.dir * isect.dist;
                isect.dist += EPS_RAY;

                LightBSDF bsdf(ray, isect, mScene);
                if(!bsdf.IsValid())
                    break;

                // Update MIS constants
                {
                    // Infinite lights use MIS based on solid angle instead of
                    // area, so we do not want distance there
                    if(lightSample.mPathLength > 1 || lightSample.mIsFiniteLight == 1)
                        lightSample.d0 *= Mis(Sqr(isect.dist));

                    lightSample.d0   /= Mis(std::abs(bsdf.CosThetaFix()));
                    lightSample.d1vc /= Mis(std::abs(bsdf.CosThetaFix()));
                    lightSample.d1vm /= Mis(std::abs(bsdf.CosThetaFix()));
                }

                // Store vertex, unless BSDF is purely specular, which prevents
                // vertex connections and merging
                if(!bsdf.IsDelta() && (mUseVC || mUseVM))
                {
                    LightVertex lightVertex;
                    lightVertex.mHitpoint   = hitPoint;
                    lightVertex.mThroughput = lightSample.mThroughput;
                    lightVertex.mPathLength = lightSample.mPathLength;
                    lightVertex.mBsdf       = bsdf;

                    lightVertex.d0   = lightSample.d0;
                    lightVertex.d1vc = lightSample.d1vc;
                    lightVertex.d1vm = lightSample.d1vm;

                    mLightVertices.push_back(lightVertex);
                }

                // Connect to camera, unless BSDF is purely specular
                if(!bsdf.IsDelta() && (mUseVC || mLightTraceOnly))
                {
                    if(lightSample.mPathLength + 1 >= mMinPathLength)
                        ConnectToCamera(lightSample, hitPoint, bsdf);
                }

                // We will now extend by the bounce (1) and then
                // we need 1 more segment to reach camera.
                // If that is too long, quit
                if(lightSample.mPathLength + 2 > mMaxPathLength)
                    break;

                if(!BounceSample(bsdf, hitPoint, lightSample))
                    break;
            }

            mPathEnds[pathIdx] = (int)mLightVertices.size();
        }

        //////////////////////////////////////////////////////////////////////////
        // Build hash grid
        //////////////////////////////////////////////////////////////////////////

        // Only build grid when merging (VCM, BPM, and Ppm)
        if(mUseVM)
        {
            // The number of cells is somewhat arbitrary, but seems to work ok
            mHashGrid.Reserve(pathCount);
            mHashGrid.Build(mLightVertices, radius);
        }

        //////////////////////////////////////////////////////////////////////////
        // Generate camera paths
        //////////////////////////////////////////////////////////////////////////

        // Light tracing does not use any camera vertices at all
        for(int pathIdx = 0; (pathIdx < pathCount) && (!mLightTraceOnly); ++pathIdx)
        {
            PathElement cameraSample;
            const Vec2f screenSample = GenerateCameraSample(pathIdx, cameraSample);
            Vec3f color(0);

            //////////////////////////////////////////////////////////////////////
            // Trace camera path
            for(;; ++cameraSample.mPathLength)
            {
                // We offset ray origin instead of setting tmin due to numeric
                // issues in ray-sphere intersection. The isect.dist has to be
                // extended by this EPS_RAY after hit point is determined
                Ray ray(cameraSample.mOrigin + cameraSample.mDirection * EPS_RAY,
                    cameraSample.mDirection, 0);

                Isect isect(1e36f);

                if(!mScene.Intersect(ray, isect))
                {
                    if(mScene.GetBackground() != NULL)
                    {
                        if(cameraSample.mPathLength >= mMinPathLength)
                        {
                            color += cameraSample.mThroughput *
                                LightOnHit(mScene.GetBackground(), cameraSample,
                                Vec3f(0), ray.dir);
                        }
                    }

                    break;
                }

                const Vec3f hitPoint = ray.org + ray.dir * isect.dist;
                isect.dist += EPS_RAY;

                CameraBSDF bsdf(ray, isect, mScene);
                if(!bsdf.IsValid())
                    break;

                // Update MIS constants
                cameraSample.d0   *= Mis(Sqr(isect.dist));
                cameraSample.d0   /= Mis(std::abs(bsdf.CosThetaFix()));
                cameraSample.d1vc /= Mis(std::abs(bsdf.CosThetaFix()));
                cameraSample.d1vm /= Mis(std::abs(bsdf.CosThetaFix()));

                // directly hit some light
                // lights do not reflect light, so we stop after this
                if(isect.lightID >= 0)
                {
                    const AbstractLight *light = mScene.GetLightPtr(isect.lightID);
                
                    if(cameraSample.mPathLength >= mMinPathLength)
                    {
                        color += cameraSample.mThroughput *
                            LightOnHit(light, cameraSample, hitPoint, ray.dir);
                    }
                    
                    break;
                }

                // Everything else needs at least one more path segment
                if(cameraSample.mPathLength >= mMaxPathLength)
                    break;

                ////////////////////////////////////////////////////////////////
                // Vertex connection: connect to lights
                if(!bsdf.IsDelta() && mUseVC)
                {
                    if(cameraSample.mPathLength + 1>= mMinPathLength)
                    {
                        color += cameraSample.mThroughput *
                            DirectIllumination(cameraSample, hitPoint, bsdf);
                    }
                }

                ////////////////////////////////////////////////////////////////
                // Vertex connection: connect to light vertices
                if(!bsdf.IsDelta() && mUseVC)
                {
                    // Each light path is assigned to one eye path, as in
                    // traditional BPT. This gives range in which are the light
                    // vertices corresponding to the current eye path.
                    // It is also possible to connect to vertices
                    // from any light path, but MIS should be revisited.
                    const Vec2i range(
                        (pathIdx == 0) ? 0 : mPathEnds[pathIdx-1],
                        mPathEnds[pathIdx]);

                    for(int i=range.x; i < range.y; i++)
                    {
                        const LightVertex &lightVertex = mLightVertices[i];
                        if(lightVertex.mPathLength + 1 +
                           cameraSample.mPathLength < mMinPathLength)
                            continue;

                        // light vertices are stored in increasing path length
                        // order, once we go above the max path length, we can
                        // skip the rest
                        if(lightVertex.mPathLength + 1 +
                           cameraSample.mPathLength > mMaxPathLength)
                            break;

                        color += cameraSample.mThroughput * lightVertex.mThroughput *
                            ConnectVertices(lightVertex, bsdf, hitPoint, cameraSample);
                    }
                }

                ////////////////////////////////////////////////////////////////
                // Vertex merging: merge with light vertices
                if(!bsdf.IsDelta() && mUseVM)
                {
                    RangeQuery query(*this, hitPoint, bsdf, cameraSample);
                    mHashGrid.Process(mLightVertices, query);
                    color += cameraSample.mThroughput * mVmNormalization * query.GetContrib();

                    // PPM merges only on first non-specular bounce
                    if(mPpm)
                        break;
                }

                if(!BounceSample(bsdf, hitPoint, cameraSample))
                    break;
            }

            mFramebuffer.AddColor(screenSample, color);
        }

        mIterations++;
    }

private:

    // Mis power, we use balance heuristic
    float Mis(float aPdf) const
    {
        //return std::pow(aPdf, /*power*/);
        return aPdf;
    }

    //////////////////////////////////////////////////////////////////////////
    // Camera tracing methods
    //////////////////////////////////////////////////////////////////////////

    // Generates new camera sample given a pixel index
    Vec2f GenerateCameraSample(
        const int   aPixelIndex,
        PathElement &oCameraSample)
    {
        const Camera &camera    = mScene.mCamera;
        const int resX = int(camera.mResolution.x);
        const int resY = int(camera.mResolution.y);

        // Determine pixel (x, y)
        const int x = aPixelIndex % resX;
        const int y = aPixelIndex / resX;

        // Jitter pixel position
        const Vec2f sample = Vec2f(float(x), float(y)) + mRng.GetVec2f();

        // Generate primary ray and find its pdf
        const Ray   primaryRay = camera.GenerateRay(sample);
        const float cosTheta   = Dot(camera.mForward, primaryRay.dir);
        const float cameraPdfW = 1.f / (cosTheta * cosTheta * cosTheta *
            camera.mPixelArea * mScreenPixelCount);

        oCameraSample.mOrigin       = primaryRay.org;
        oCameraSample.mDirection    = primaryRay.dir;
        oCameraSample.mThroughput   = Vec3f(1);

        oCameraSample.mPathLength   = 1;
        oCameraSample.mSpecularPath = 1;

        oCameraSample.d0            = Mis(1.f / cameraPdfW);
        oCameraSample.d1vc          = 0;
        oCameraSample.d1vm          = 0;

        return sample;
    }

    // Returns (unweighted) radiance when ray hits a light source.
    // Can be used for both Background and Area lights.
    //
    // For Background lights:
    //    Has to be called BEFORE updating the MIS quantities.
    //    Value of aHitpoint is irrelevant (passing Vec3f(0))
    //
    // For Area lights:
    //    Has to be called AFTER updating the MIS quantities.
    Vec3f LightOnHit(
        const AbstractLight *aLight,
        const PathElement   &aCameraSample,
        const Vec3f         &aHitpoint,
        const Vec3f         &aRayDirection) const
    {
        // We sample lights uniformly
        const int   lightCount    = mScene.GetLightCount();
        const float lightPickProb = 1.f / lightCount;

        float directPdfA, emissionPdfW;
        const Vec3f radiance = aLight->GetRadiance(mScene.mSceneSphere,
            aRayDirection, aHitpoint, &directPdfA, &emissionPdfW);

        if(radiance.IsZero())
            return Vec3f(0);

        // If we see background directly from camera, no weighting is required
        if(aCameraSample.mPathLength == 1)
            return radiance;

        // When using only vertex merging, we want purely specular paths
        // to give radiance (cannot get it otherwise). Rest is handled
        // by merging and we should return 0.
        if(mUseVM && !mUseVC)
            return aCameraSample.mSpecularPath ? radiance : Vec3f(0);

        directPdfA   *= lightPickProb;
        emissionPdfW *= lightPickProb;

        // If the last hit was specular, then d0 == 0.
        const float wCamera = Mis(directPdfA) * aCameraSample.d0 +
            Mis(emissionPdfW) * aCameraSample.d1vc;

        const float misWeight = 1.f / (1.f + wCamera);
        return misWeight * radiance;
    }

    // Connects camera sample to randomly chosen sample, returns (unweighted) radiance.
    // Has to be called AFTER updating the MIS quantities.
    Vec3f DirectIllumination(
        const PathElement &aCameraSample,
        const Vec3f       &aHitpoint,
        const CameraBSDF  &aBsdf)
    {
        // We sample lights uniformly
        const int   lightCount    = mScene.GetLightCount();
        const float lightPickProb = 1.f / lightCount;

        const int   lightID       = int(mRng.GetFloat() * lightCount);
        const Vec2f rndPosSamples = mRng.GetVec2f();

        const AbstractLight *light = mScene.GetLightPtr(lightID);

        Vec3f directionToLight;
        float distance;
        float directPdfW, emissionPdfW, cosAtLight;
        const Vec3f radiance = light->Illuminate(mScene.mSceneSphere, aHitpoint,
            rndPosSamples, directionToLight, distance, directPdfW,
            &emissionPdfW, &cosAtLight);

        // if radiance == 0, other values are undefined, so have to early exit
        if(radiance.IsZero())
            return Vec3f(0);

        float bsdfDirPdfW, bsdfRevPdfW, cosToLight;
        const Vec3f bsdfFactor = aBsdf.Evaluate(mScene,
            directionToLight, cosToLight, &bsdfDirPdfW, &bsdfRevPdfW);

        if(bsdfFactor.IsZero())
            return Vec3f(0);

        const float continuationProbability = aBsdf.ContinuationProb();
        
        // If the light is delta light, we can never hit it
        // by BSDF sampling, so the probability of this path is 0
        bsdfDirPdfW *= light->IsDelta() ? 0.f : continuationProbability;

        bsdfRevPdfW *= continuationProbability;

        // What we ultimately want to do is
        //    ratio = emissionPdfA / directPdfA
        // What we have is
        //    emissionPdfW, and directPdfW = directPdfA * dist^2 / cosAtLight
        // Expanding we get
        //    emissionPdfA = emissionPdfW * cosToLight / dist^2
        //    directPdfA   = directPdfW * cosAtLight / dist^2
        //    ratio = (emissionPdfW * cosToLight / dist^2) / (directPdfW * cosAtLight / dist^2)
        //    ratio = (emissionPdfW * cosToLight) / (directPdfW * cosAtLight)

        // Also note that we multiply by lightPickProb only for wLight,
        // as it cancels out in wCamera

        const float wLight  = Mis(bsdfDirPdfW / (lightPickProb * directPdfW));
        const float wCamera = Mis(emissionPdfW * cosToLight / (directPdfW * cosAtLight)) * (
            mMisVmWeightFactor + aCameraSample.d0 + aCameraSample.d1vc * Mis(bsdfRevPdfW));
        const float misWeight = 1.f / (wLight + 1.f + wCamera);

        const Vec3f contrib =
            (misWeight * cosToLight / (lightPickProb * directPdfW)) * (radiance * bsdfFactor);

        if(contrib.IsZero() || mScene.Occluded(aHitpoint, directionToLight, distance))
            return Vec3f(0);

        return contrib;
    }

    // Connects camera and light sample. Result is not multiplied by throughputs.
    // Has to be called AFTER updating MIS constants. 'direction' is FROM
    // camera TO light vertex
    Vec3f ConnectVertices(
        const LightVertex &aLightVertex,
        const CameraBSDF  &aCameraBsdf,
        const Vec3f       &aCameraHitpoint,
        const PathElement &aCameraSample) const
    {
        // Get the connection
        Vec3f direction   = aLightVertex.mHitpoint - aCameraHitpoint;
        const float dist2 = direction.LenSqr();
        float  distance   = std::sqrt(dist2);
        direction        /= distance;

        // Evaluate BSDF at camera vertex
        float cosCamera, cameraBsdfDirPdfW, cameraBsdfRevPdfW;
        const Vec3f cameraBsdfFactor = aCameraBsdf.Evaluate(
            mScene, direction, cosCamera, &cameraBsdfDirPdfW,
            &cameraBsdfRevPdfW);

        if(cameraBsdfFactor.IsZero())
            return Vec3f(0);

        // Camera continuation probability (for Russian roulette)
        const float cameraCont = aCameraBsdf.ContinuationProb();
        cameraBsdfDirPdfW *= cameraCont;
        cameraBsdfRevPdfW *= cameraCont;

        // Evaluate BSDF at light vertex
        float cosLight, lightBsdfDirPdfW, lightBsdfRevPdfW;
        const Vec3f lightBsdfFactor = aLightVertex.mBsdf.Evaluate(
            mScene, -direction, cosLight, &lightBsdfDirPdfW,
            &lightBsdfRevPdfW);

        if(lightBsdfFactor.IsZero())
            return Vec3f(0);

        // Light continuation probability (for Russian roulette)
        const float lightCont = aLightVertex.mBsdf.ContinuationProb();
        lightBsdfDirPdfW *= lightCont;
        lightBsdfRevPdfW *= lightCont;

        // Compute geometry term
        const float geometryTerm = cosLight * cosCamera / dist2;
        if(geometryTerm < 0)
            return Vec3f(0);

        // Convert pdfs to area pdf
        const float cameraBsdfDirPdfA = PdfWtoA(cameraBsdfDirPdfW, distance, cosLight);
        const float lightBsdfDirPdfA  = PdfWtoA(lightBsdfDirPdfW,  distance, cosCamera);

        // MIS weights
        const float wLight = Mis(cameraBsdfDirPdfA) * (
            mMisVmWeightFactor + aLightVertex.d0 + aLightVertex.d1vc * Mis(lightBsdfRevPdfW));
        const float wCamera = Mis(lightBsdfDirPdfA) * (
            mMisVmWeightFactor + aCameraSample.d0 + aCameraSample.d1vc * Mis(cameraBsdfRevPdfW));

        const float misWeight = 1.f / (wLight + 1.f + wCamera);

        const Vec3f contrib = (misWeight * geometryTerm) * cameraBsdfFactor * lightBsdfFactor;

        if(contrib.IsZero() || mScene.Occluded(aCameraHitpoint, direction, distance))
            return Vec3f(0);

        return contrib;
    }

    //////////////////////////////////////////////////////////////////////////
    // Light tracing methods
    //////////////////////////////////////////////////////////////////////////

    // Samples light emission
    void GenerateLightSample(PathElement &oLightSample)
    {
        // We sample lights uniformly
        const int   lightCount    = mScene.GetLightCount();
        const float lightPickProb = 1.f / lightCount;

        const int   lightID       = int(mRng.GetFloat() * lightCount);
        const Vec2f rndDirSamples = mRng.GetVec2f();
        const Vec2f rndPosSamples = mRng.GetVec2f();

        const AbstractLight *light = mScene.GetLightPtr(lightID);

        float emissionPdfW, directPdfW, cosLight;
        oLightSample.mThroughput = light->Emit(mScene.mSceneSphere, rndDirSamples, rndPosSamples,
            oLightSample.mOrigin, oLightSample.mDirection,
            emissionPdfW, &directPdfW, &cosLight);

        emissionPdfW *= lightPickProb;
        directPdfW   *= lightPickProb;

        oLightSample.mThroughput    /= emissionPdfW;
        oLightSample.mPathLength    = 1;
        oLightSample.mIsFiniteLight = light->IsFinite() ? 1 : 0;

        oLightSample.d0 = Mis(directPdfW / emissionPdfW);

        if(!light->IsDelta())
        {
            const float usedCosLight = light->IsFinite() ? cosLight : 1.f;
            oLightSample.d1vc = Mis(usedCosLight / emissionPdfW);
        }
        else
        {
            oLightSample.d1vc = 0.f;
        }

        oLightSample.d1vm = oLightSample.d1vc * mMisVcWeightFactor;
    }

    // Computes contribution of light sample to camera by splatting is onto the
    // framebuffer. Multiplies by throughput (obviously, as nothing is returned).
    void ConnectToCamera(
        const PathElement &aLightSample,
        const Vec3f       &aHitpoint,
        const LightBSDF   &aBsdf)
    {
        const Camera &camera    = mScene.mCamera;
        Vec3f directionToCamera = camera.mPosition - aHitpoint;

        // Check point is in front of camera
        if(Dot(camera.mForward, -directionToCamera) <= 0.f)
            return;

        // Check it projects to the screen (and where)
        const Vec2f imagePos = camera.WorldToRaster(aHitpoint);
        if(!camera.CheckRaster(imagePos))
            return;

        // Compute distance and normalize direction to camera
        const float distEye2 = directionToCamera.LenSqr();
        const float distance            = std::sqrt(distEye2);
        directionToCamera  /= distance;

        // Get the BSDF
        float cosToCamera, bsdfDirPdfW, bsdfRevPdfW;
        const Vec3f bsdfFactor = aBsdf.Evaluate(mScene,
            directionToCamera, cosToCamera, &bsdfDirPdfW, &bsdfRevPdfW);

        if(bsdfFactor.IsZero())
            return;

        bsdfRevPdfW *= aBsdf.ContinuationProb();

        // PDF of ray from camera hitting here (w.r.t. real resolution)
        const float cosAtCamera = Dot(camera.mForward, -directionToCamera);
        const float cameraPdfW = 1.f / (cosAtCamera * cosAtCamera * cosAtCamera *
            camera.mPixelArea);
        const float cameraPdfA = PdfWtoA(cameraPdfW, distance, cosToCamera);

        // MIS weights, we need cameraPdfA w.r.t. normalized device coordinate,
        // so we divide by (resolution.x * resolution.y)
        const float wLight = Mis(cameraPdfA / mScreenPixelCount) * (
            mMisVmWeightFactor + aLightSample.d0 + aLightSample.d1vc * Mis(bsdfRevPdfW));

        const float misWeight = mLightTraceOnly ? 1.f : (1.f / (wLight + 1.f));

        const float fluxToRadianceFactor = cameraPdfA;
        const Vec3f contrib = misWeight * fluxToRadianceFactor * bsdfFactor;

        if(!contrib.IsZero())
        {
            if(mScene.Occluded(aHitpoint, directionToCamera, distance))
                return;

            mFramebuffer.AddColor(imagePos,
                contrib * aLightSample.mThroughput / mLightPathCount);
        }
    }

    // Bounces camera/light sample according to BSDF, returns false for termination
    template<bool tLightSample>
    bool BounceSample(
        const BSDF<tLightSample> &aBsdf,
        const Vec3f              &aHitPoint,
        PathElement              &aoPathSample)
    {
        // x,y for direction, z for component. No rescaling happens
        Vec3f rndTriplet  = mRng.GetVec3f();
        float bsdfDirPdfW, cosThetaOut;
        uint  sampledEvent;

        Vec3f bsdfFactor = aBsdf.Sample(mScene, rndTriplet, aoPathSample.mDirection,
            bsdfDirPdfW, cosThetaOut, &sampledEvent);

        if(bsdfFactor.IsZero())
            return false;

        // If we sampled specular event, then the reverse probability
        // cannot be evaluated, but we know it is exactly the same as
        // direct probability, so just set it. If non-specular event happened,
        // we evaluate the pdf
        float bsdfRevPdfW = bsdfDirPdfW;
        if((sampledEvent & LightBSDF::kSpecular) == 0)
            bsdfRevPdfW = aBsdf.Pdf(mScene, aoPathSample.mDirection, true);

        // Russian roulette
        const float contProb = aBsdf.ContinuationProb();
        if(mRng.GetFloat() > contProb)
            return false;

        bsdfDirPdfW *= contProb;
        bsdfRevPdfW *= contProb;

        // New MIS weights
        if(sampledEvent & LightBSDF::kSpecular)
        {
            aoPathSample.d0 = 0.f;

            aoPathSample.d1vc *=
                Mis(cosThetaOut / bsdfDirPdfW) * Mis(bsdfRevPdfW);

            aoPathSample.d1vm *=
                Mis(cosThetaOut / bsdfDirPdfW) * Mis(bsdfRevPdfW);

            aoPathSample.mSpecularPath &= 1;
        }
        else
        {
            aoPathSample.d1vc = Mis(cosThetaOut / bsdfDirPdfW) * (
                aoPathSample.d1vc * Mis(bsdfRevPdfW) +
                aoPathSample.d0 + mMisVmWeightFactor);

            aoPathSample.d1vm = Mis(cosThetaOut / bsdfDirPdfW) * (
                aoPathSample.d1vm * Mis(bsdfRevPdfW) +
                aoPathSample.d0 * mMisVcWeightFactor + 1.f);

            aoPathSample.d0 = Mis(1.f / bsdfDirPdfW);

            aoPathSample.mSpecularPath &= 0;
        }

        aoPathSample.mOrigin  = aHitPoint;
        aoPathSample.mThroughput *= bsdfFactor * (cosThetaOut / bsdfDirPdfW);
        
        return true;
    }

private:

    bool  mUseVM;             // Vertex merging (of some form) is used
    bool  mUseVC;             // Vertex connection (BPT) is used
    bool  mLightTraceOnly;    // Do only light tracing
    bool  mPpm;               // Do PPM, same terminates camera after first merge

    float mRadiusAlpha;       // Radius reduction rate parameter
    float mBaseRadius;        // Initial merging radius
    float mMisVmWeightFactor; // Weight of vertex merging (used in VC)
    float mMisVcWeightFactor; // Weight of vertex connection (used in VM)
    float mScreenPixelCount;  // Number of pixels
    float mLightPathCount;    // Number of light paths
    float mVmNormalization;   // 1 / (Pi * radius^2 * light_path_count)

    std::vector<LightVertex> mLightVertices; //!< Stored light vertices

    // For light path belonging to pixel index [x] it stores
    // where it's light vertices end (begin is at [x-1])
    std::vector<int> mPathEnds;
    HashGrid         mHashGrid;

    Rng              mRng;
};

#endif //__VERTEXCM_HXX__
