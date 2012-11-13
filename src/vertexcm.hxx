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

        float dVCM; // MIS quantity used for vertex connection and merging
        float dVC;  // MIS quantity used for vertex connection
        float dVM;  // MIS quantity used for vertex merging
    };

    // Path vertex, used for merging and connection
    template<bool tFromLight>
    struct PathVertex
    {
        Vec3f mHitpoint;   // Position of the vertex
        Vec3f mThroughput; // Path throughput (including emission)
        uint  mPathLength; // Number of segments between source and vertex

        // Stores all required local information, including incoming direction.
        BSDF<tFromLight> mBsdf;

        // We compute MIS in a cumulative fashion. 1 variable is used,
        // plus 1 for each used method (connection, merging).
        // Please see the accompanying writeup for derivation.

        float dVCM; // MIS quantity used for vertex connection and merging
        float dVC;  // MIS quantity used for vertex connection
        float dVM;  // MIS quantity used for vertex merging

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
            // Reject if full path length below/above min/max path length
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

            // Partial light sub-path MIS weight [tech. rep. (38)]
            const float wLight = aLightVertex.dVCM * mVertexCM.mMisVcWeightFactor +
                aLightVertex.dVM * mVertexCM.Mis(cameraBsdfDirPdfW);

            // Partial eye sub-path MIS weight [tech. rep. (39)]
            const float wCamera = mCameraSample.dVCM * mVertexCM.mMisVcWeightFactor +
                mCameraSample.dVM * mVertexCM.Mis(cameraBsdfRevPdfW);

            // Full path MIS weight [tech. rep. (37)]. No MIS for PPM
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
        // No MIS weights (dVCM, dVM, dVC all ignored)
        kLightTrace = 0,

        // Camera and light vertices merged on first non-specular camera bounce.
        // Cannot handle mixed specular + non-specular materials.
        // No MIS weights (dVCM, dVM, dVC all ignored)
        kPpm,

        // Camera and light vertices merged on along full path.
        // dVCM and dVM used for MIS
        kBpm,

        // Standard bidirectional path tracing
        // dVCM and dVC used for MIS
        kBpt,

        // Vertex connection and mering
        // dVCM, dVM, and dVC used for MIS
        kVcm
    };

public:

    VertexCM(
        const Scene&  aScene,
        AlgorithmType aAlgorithm,
        const float   aRadiusFactor,
        const float   aRadiusAlpha,
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
                        "*WARNING* Our PPM implementation cannot handle materials mixing\n"
                        "Specular and NonSpecular BSDFs. The extension would be\n"
                        "fairly straightforward. In BounceSample for CameraSample\n"
                        "limit the considered events to Specular only.\n"
                        "Merging will use non-specular components, bounce will be specular.\n"
                        "If there is no specular component, the ray will terminate.\n\n");

                    printf("We are now switching from *PPM* to *BPM*, which can handle the scene\n\n");

                    mPpm = false;
                    break;
                }
            }
        }

        mBaseRadius  = aRadiusFactor * mScene.mSceneSphere.mSceneRadius;
        mRadiusAlpha = aRadiusAlpha;
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

        // MIS weight constant [tech. rep. (20)], with n_VC = 1 and n_VM = mLightPathCount
        const float etaVCM = (PI_F * radiusSqr) * mLightPathCount;
        mMisVmWeightFactor = mUseVM ? Mis(etaVCM)       : 0.f;
        mMisVcWeightFactor = mUseVC ? Mis(1.f / etaVCM) : 0.f;

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
                // Offset ray origin instead of setting tmin due to numeric
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

                // Update the MIS quantities before storing them at the vertex.
                // These updates follow the initialization in GenerateLightSample() or
                // BounceSample(), and together implement equations [tech. rep. (31)-(33)]
		// or [tech. rep. (34)-(36)], respectively.
                {
                    // Infinite lights use MIS handled via solid angle integration,
                    // so do not divide by the distance for such lights [tech. rep. Section 5.1]
                    if(lightSample.mPathLength > 1 || lightSample.mIsFiniteLight == 1)
                        lightSample.dVCM *= Mis(Sqr(isect.dist));

                    lightSample.dVCM /= Mis(std::abs(bsdf.CosThetaFix()));
                    lightSample.dVC  /= Mis(std::abs(bsdf.CosThetaFix()));
                    lightSample.dVM  /= Mis(std::abs(bsdf.CosThetaFix()));
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

                    lightVertex.dVCM = lightSample.dVCM;
                    lightVertex.dVC  = lightSample.dVC;
                    lightVertex.dVM  = lightSample.dVM;

                    mLightVertices.push_back(lightVertex);
                }

                // Connect to camera, unless BSDF is purely specular
                if(!bsdf.IsDelta() && (mUseVC || mLightTraceOnly))
                {
                    if(lightSample.mPathLength + 1 >= mMinPathLength)
                        ConnectToCamera(lightSample, hitPoint, bsdf);
                }

                // Terminate if the path would become too long after scattering
                if(lightSample.mPathLength + 2 > mMaxPathLength)
                    break;

                // Continue random walk
                if(!BounceSample(bsdf, hitPoint, lightSample))
                    break;
            }

            mPathEnds[pathIdx] = (int)mLightVertices.size();
        }

        //////////////////////////////////////////////////////////////////////////
        // Build hash grid
        //////////////////////////////////////////////////////////////////////////

        // Only build grid when merging (VCM, BPM, and PPM)
        if(mUseVM)
        {
            // The number of cells is somewhat arbitrary, but seems to work ok
            mHashGrid.Reserve(pathCount);
            mHashGrid.Build(mLightVertices, radius);
        }

        //////////////////////////////////////////////////////////////////////////
        // Generate camera paths
        //////////////////////////////////////////////////////////////////////////

        // Unless rendering with traditional light tracing
        for(int pathIdx = 0; (pathIdx < pathCount) && (!mLightTraceOnly); ++pathIdx)
        {
            PathElement cameraSample;
            const Vec2f screenSample = GenerateCameraSample(pathIdx, cameraSample);
            Vec3f color(0);

            //////////////////////////////////////////////////////////////////////
            // Trace camera path
            for(;; ++cameraSample.mPathLength)
            {
                // Offset ray origin instead of setting tmin due to numeric
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

                // Update the MIS quantities, following the initialization in
		// GenerateLightSample() or BounceSample(). Implement equations
		// [tech. rep. (31)-(33)] or [tech. rep. (34)-(36)], respectively.
                {
                    cameraSample.dVCM *= Mis(Sqr(isect.dist));
                    cameraSample.dVCM /= Mis(std::abs(bsdf.CosThetaFix()));
                    cameraSample.dVC  /= Mis(std::abs(bsdf.CosThetaFix()));
                    cameraSample.dVM  /= Mis(std::abs(bsdf.CosThetaFix()));
                }

                // Light source has been hit; terminate afterwards, since
                // our light sources do not have reflective properties
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

                // Terminate if eye sub-path is too long for connections or merging
                if(cameraSample.mPathLength >= mMaxPathLength)
                    break;

                ////////////////////////////////////////////////////////////////
                // Vertex connection: Connect to a light source
                if(!bsdf.IsDelta() && mUseVC)
                {
                    if(cameraSample.mPathLength + 1>= mMinPathLength)
                    {
                        color += cameraSample.mThroughput *
                            DirectIllumination(cameraSample, hitPoint, bsdf);
                    }
                }

                ////////////////////////////////////////////////////////////////
                // Vertex connection: Connect to light vertices
                if(!bsdf.IsDelta() && mUseVC)
                {
                    // For VC, each light sub-path is assigned to a particular eye
                    // sub-path, as in traditional BPT. It is also possible to
                    // connect to vertices from any light path, but MIS should
                    // be revisited.
                    const Vec2i range(
                        (pathIdx == 0) ? 0 : mPathEnds[pathIdx-1],
                        mPathEnds[pathIdx]);

                    for(int i = range.x; i < range.y; i++)
                    {
                        const LightVertex &lightVertex = mLightVertices[i];

                        if(lightVertex.mPathLength + 1 +
                           cameraSample.mPathLength < mMinPathLength)
                            continue;

                        // Light vertices are stored in increasing path length
                        // order; once we go above the max path length, we can
                        // skip the rest
                        if(lightVertex.mPathLength + 1 +
                           cameraSample.mPathLength > mMaxPathLength)
                            break;

                        color += cameraSample.mThroughput * lightVertex.mThroughput *
                            ConnectVertices(lightVertex, bsdf, hitPoint, cameraSample);
                    }
                }

                ////////////////////////////////////////////////////////////////
                // Vertex merging: Merge with light vertices
                if(!bsdf.IsDelta() && mUseVM)
                {
                    RangeQuery query(*this, hitPoint, bsdf, cameraSample);
                    mHashGrid.Process(mLightVertices, query);
                    color += cameraSample.mThroughput * mVmNormalization * query.GetContrib();

                    // PPM merges only on first non-specular bounce
                    if(mPpm) break;
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
        const Camera &camera = mScene.mCamera;
        const int resX = int(camera.mResolution.x);
        const int resY = int(camera.mResolution.y);

        // Determine pixel (x, y)
        const int x = aPixelIndex % resX;
        const int y = aPixelIndex / resX;

        // Jitter pixel position
        const Vec2f sample = Vec2f(float(x), float(y)) + mRng.GetVec2f();

        // Generate ray
        const Ray primaryRay = camera.GenerateRay(sample);

        // Pdf for sampling image plane position; uniform within the pixel in our case
        const float imagePlanePdf = 1.f / camera.mPixelArea;

        // Pdf conversion factor from area on image plane to solid angle on ray
        const float cosAtCamera   = Dot(camera.mForward, primaryRay.dir);
        const float imageToSolidAngleFactor =
            1.f / (cosAtCamera * cosAtCamera * cosAtCamera);

        // Solid angle pdf for camera ray
        const float cameraPdfW = imagePlanePdf * imageToSolidAngleFactor;

        oCameraSample.mOrigin       = primaryRay.org;
        oCameraSample.mDirection    = primaryRay.dir;
        oCameraSample.mThroughput   = Vec3f(1);

        oCameraSample.mPathLength   = 1;
        oCameraSample.mSpecularPath = 1;

        // Eye sub-path MIS quantities. Implements [tech. rep. (31)-(33)] partially.
        // The evaluation is completed after tracing the camera ray in the eye sub-path loop.
        oCameraSample.dVCM = Mis(mLightPathCount / cameraPdfW);
        oCameraSample.dVC  = 0;
        oCameraSample.dVM  = 0;

        return sample;
    }

    // Returns the radiance of a light source when hit by a random ray,
    // multiplied by MIS weight. Can be used for both Background and Area lights.
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

        // If we see light source directly from camera, no weighting is required
        if(aCameraSample.mPathLength == 1)
            return radiance;

        // When using only vertex merging, we want purely specular paths
        // to give radiance (cannot get it otherwise). Rest is handled
        // by merging and we should return 0.
        if(mUseVM && !mUseVC)
            return aCameraSample.mSpecularPath ? radiance : Vec3f(0);

        directPdfA   *= lightPickProb;
        emissionPdfW *= lightPickProb;

        // Partial eye sub-path MIS weight [tech. rep. (43)].
        // Some part of it has been already computed in BounceSample().
        // If the last hit was specular, then dVCM == 0.
        const float wCamera = Mis(directPdfA) * aCameraSample.dVCM +
            Mis(emissionPdfW) * aCameraSample.dVC;

        // Partial light sub-path weight is 0 [tech. rep. (42)].

        // Full path MIS weight [tech. rep. (37)].
        const float misWeight = 1.f / (1.f + wCamera);
        
        return misWeight * radiance;
    }

    // Connects camera vertex to randomly chosen light point.
    // Returns emitted radiance multiplied by path MIS weight.
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

        // If radiance == 0, other values are undefined, so have to early exit
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

        // Partial light sub-path MIS weight [tech. rep. (44)]
        const float wLight = Mis(bsdfDirPdfW / (lightPickProb * directPdfW));

        // Partial eye sub-path MIS weight [tech. rep. (45)]
        const float wCamera = Mis(emissionPdfW * cosToLight / (directPdfW * cosAtLight)) * (
            mMisVmWeightFactor + aCameraSample.dVCM + aCameraSample.dVC * Mis(bsdfRevPdfW));

        // Full path MIS weight [tech. rep. (37)]
        const float misWeight = 1.f / (wLight + 1.f + wCamera);

        const Vec3f contrib =
            (misWeight * cosToLight / (lightPickProb * directPdfW)) * (radiance * bsdfFactor);

        if(contrib.IsZero() || mScene.Occluded(aHitpoint, directionToLight, distance))
            return Vec3f(0);

        return contrib;
    }

    // Connects an eye and a light vertex. Result multiplied by MIS weight, but
    // not multiplied by vertex throughputs. Has to be called AFTER updating MIS
    // constants. 'direction' is FROM eye TO light vertex.
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

        // Partial light sub-path MIS weight [tech. rep. (40)]
        const float wLight = Mis(cameraBsdfDirPdfA) * (
            mMisVmWeightFactor + aLightVertex.dVCM + aLightVertex.dVC * Mis(lightBsdfRevPdfW));

        // Partial eye sub-path MIS weight [tech. rep. (41)]
        const float wCamera = Mis(lightBsdfDirPdfA) * (
            mMisVmWeightFactor + aCameraSample.dVCM + aCameraSample.dVC * Mis(cameraBsdfRevPdfW));

        // Full path MIS weight [tech. rep. (37)]
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

        // Light sub-path MIS quantities. Implements [tech. rep. (31)-(33)] partially.
        // The evaluation is completed after tracing the emission ray in the light sub-path loop.
        // Delta lights are handled as well [tech. rep. (48)-(50)].
        {
            oLightSample.dVCM = Mis(directPdfW / emissionPdfW);

            if(!light->IsDelta())
            {
                const float usedCosLight = light->IsFinite() ? cosLight : 1.f;
                oLightSample.dVC = Mis(usedCosLight / emissionPdfW);
            }
            else
            {
                oLightSample.dVC = 0.f;
            }

            oLightSample.dVM = oLightSample.dVC * mMisVcWeightFactor;
        }
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
        const float distance = std::sqrt(distEye2);
        directionToCamera   /= distance;

        // Get the BSDF
        float cosToCamera, bsdfDirPdfW, bsdfRevPdfW;
        const Vec3f bsdfFactor = aBsdf.Evaluate(mScene,
            directionToCamera, cosToCamera, &bsdfDirPdfW, &bsdfRevPdfW);

        if(bsdfFactor.IsZero())
            return;

        bsdfRevPdfW *= aBsdf.ContinuationProb();

        // Pdf for sampling image plane position; uniform within the pixel in our case
        const float imagePlanePdf = 1.f / camera.mPixelArea;
        
        // Pdf conversion factor from area on image plane to solid angle on ray
        const float cosAtCamera = Dot(camera.mForward, -directionToCamera);
        const float imageToSolidAngleFactor =
            1.f / (cosAtCamera * cosAtCamera * cosAtCamera);

        // Solid angle pdf for camera ray
        const float cameraPdfW = imagePlanePdf * imageToSolidAngleFactor;

        // Area pdf of surface point aHitpoint sampled from the camera
        const float cameraPdfA = PdfWtoA(cameraPdfW, distance, cosToCamera);

        // Partial light sub-path weight [tech. rep. (46)]. Note the division by
        // mLightPathCount, which is the number of samples this technique uses.
        // This division also appears a few lines below in the framebuffer accumulation.
        const float wLight = Mis(cameraPdfA / mLightPathCount) * (
            mMisVmWeightFactor + aLightSample.dVCM + aLightSample.dVC * Mis(bsdfRevPdfW));

        // Partial eye sub-path weight is 0 [tech. rep. (47)]

        // Full path MIS weight [tech. rep. (37)]. No MIS for traditional light tracing.
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
        // forward probability, so just set it. If non-specular event happened,
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

        // Sub-path MIS quantities for the next vertex. Only partial - the
        // evaluation is completed when the actual hit point is known,
        // i.e. after tracing the ray, in the sub-path loop.

        if(sampledEvent & LightBSDF::kSpecular)
        {
            // Specular scattering case [tech. rep. (53)-(55)] (partially, as noted above)
            aoPathSample.dVCM = 0.f;
            aoPathSample.dVC *= Mis(cosThetaOut / bsdfDirPdfW) * Mis(bsdfRevPdfW);
            aoPathSample.dVM *= Mis(cosThetaOut / bsdfDirPdfW) * Mis(bsdfRevPdfW);

            aoPathSample.mSpecularPath &= 1;
        }
        else
        {
            // Implements [tech. rep. (34)-(36)] (partially, as noted above)
            aoPathSample.dVC = Mis(cosThetaOut / bsdfDirPdfW) * (
                aoPathSample.dVC * Mis(bsdfRevPdfW) +
                aoPathSample.dVCM + mMisVmWeightFactor);

            aoPathSample.dVM = Mis(cosThetaOut / bsdfDirPdfW) * (
                aoPathSample.dVM * Mis(bsdfRevPdfW) +
                aoPathSample.dVCM * mMisVcWeightFactor + 1.f);

            aoPathSample.dVCM = Mis(1.f / bsdfDirPdfW);

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
