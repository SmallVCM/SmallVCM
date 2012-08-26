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
#include "bxdf.hxx"
#include "rng.hxx"
#include "hashgrid.hxx"

// This allows to easily turn on/off parts of algorithm,
// to evaluate relative contributions. Algorithm does not
// produce correct results unless all five are on
#define DIR_CON   1 //!< Direct connection of light particles to camera
#define ON_HIT    1 //!< Contribution when area light/background hit randomly
#define DIR_LIGHT 1 //!< Direct connection of camera particles to lights
#define VC        1 //!< Connection between light and camera particles
#define VM        1 //!< Merging between light and camera particles

class VertexCM : public AbstractRenderer
{
    /* \brief Path element is used for tracing
     *
     * The sole point of this structure is to make carrying around
     * the ray baggage easier.
     */
    struct PathElement
    {
        Vec3f mOrigin;     //!< Path origin
        Vec3f mDirection;  //!< Where to go next
        Vec3f mWeight;     //!< Path weight
        uint  mPathLength    : 20; //!< Number of path segments, including this
        uint  mIsFiniteLight :  1; //!< Just generate by finite light
        uint  mSpecularPath  :  1; //!< All bounces so far were specular

        // We compute MIS in a cumulative fashion. 1 variable is used,
        // plus 1 for each used method (connection, merging).
        // Please see the accompanying writeup for derivation.
        float d0;   //!< Common helper variable for MIS
        float d1vc; //!< Helper variable for vertex connection MIS
        float d1vm; //!< Helper variable for vertex merging MIS
    };

    /* \brief Path vertex, used for merging and connection
     *
     */
    template<bool tFromLight>
    struct PathVertex
    {
        Vec3f mHitpoint;   //!< Position of the vertex
        Vec3f mWeight;     //!< Weight (multiply contribution)
        uint  mPathLength; //!< How many segments between source and vertex

        /* \brief BXDF at vertex position
         *
         * This stores all required local information, including incoming
         * direction.
         */
        BXDF<tFromLight> mBxdf;
        // We compute MIS in a cumulative fashion. 1 variable is used,
        // plus 1 for each used method (connection, merging).
        // Please see the accompanying writeup for derivation.
        float d0;   //!< Common helper variable for MIS
        float d1vc; //!< Helper variable for vertex connection MIS
        float d1vm; //!< Helper variable for vertex merging MIS

        // Used by HashGrid
        const Vec3f& GetPosition() const { return mHitpoint; }
    };

    typedef PathVertex<false> CameraVertex;
    typedef PathVertex<true>  LightVertex;

    typedef BXDF<false>       CameraBxdf;
    typedef BXDF<true>        LightBxdf;

    /* \brief Range query used for Ppm, Bpm, and Vcm
     *
     * Is used by HashGrid, when a particle is found within
     * range (given by hash grid), Process() is called
     * and vertex merge is performed. BXDF of the camera
     * particle is used.
     */
    class RangeQuery
    {
    public:
        RangeQuery(
            const VertexCM    &aVertexCM,
            const Vec3f       &aCameraPosition,
            const CameraBxdf  &aCameraBxdf,
            const PathElement &aCameraSample)
            : mVertexCM(aVertexCM),
            mCameraPosition(aCameraPosition),
            mCameraBxdf(aCameraBxdf),
            mCameraSample(aCameraSample),
            mContrib(0)
        {

        }

        const Vec3f& GetPosition() const { return mCameraPosition; }

        void Process(const LightVertex& aLightVertex)
        {
            if(aLightVertex.mPathLength +
                mCameraSample.mPathLength > mVertexCM.mMaxPathLength)
                return;

            // Retrieve light' incoming direction in world coordinates
            const Vec3f lightDirection = aLightVertex.mBxdf.WorldOmegaFix();

            float cosCamera, cameraBrdfDirPdfW, cameraBrdfRevPdfW;
            const Vec3f cameraBrdfFactor = mCameraBxdf.EvaluateBrdfPdfW(
                mVertexCM.mScene, lightDirection, cosCamera, &cameraBrdfDirPdfW,
                &cameraBrdfRevPdfW);

            if(cameraBrdfFactor.IsZero())
                return;

            const float wLight = aLightVertex.d0 * mVertexCM.mMisVcWeightFactor +
                aLightVertex.d1vm * mVertexCM.Mis(cameraBrdfDirPdfW);
            const float wCamera = mCameraSample.d0 * mVertexCM.mMisVcWeightFactor +
                mCameraSample.d1vm * mVertexCM.Mis(cameraBrdfRevPdfW);

            float weight = 1.f;

            // Ppm merges, but does not have MIS weights
            if(!mVertexCM.mPpm)
                weight = 1.f / (wLight + 1.f + wCamera);

            mContrib += weight * cameraBrdfFactor * aLightVertex.mWeight;
        }

        const Vec3f& GetContrib() const { return mContrib; }
    private:
        const VertexCM    &mVertexCM;
        const Vec3f       &mCameraPosition;
        const CameraBxdf  &mCameraBxdf;
        const PathElement &mCameraSample;
        Vec3f             mContrib;
    };
public:
    enum AlgorithmType
    {
        // light particles contribute to camera,
        // No MIS weights (d0, d1vm, d1vc all ignored)
        kLightTrace = 0,
        // Camera and light particles merged on first non-specular camera bounce.
        // Cannot handle mixed specular + non-specular materials.
        // No MIS weights (d0, d1vm, d1vc all ignored)
        kPpm,
        // Camera and light particles merged on along full path.
        // d0 and d1vm used for MIS
        kBpm,
        // Standard bidirection path tracing
        // d0 and d1vc used for MIS
        kBpt,
        // Vertex connection and mering
        // d0, d1vm, and d1vc used for MIS
        kVcm
    };
public:
    VertexCM(const Scene& aScene, AlgorithmType aAlgorithm,
        int aSeed = 1234) : AbstractRenderer(aScene), mRng(aSeed)
    {
        mLightTraceOnly = false;
        mUseVC          = false;
        mUseVM          = false;
        mPpm            = false;
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
            for(int i=0; i<mScene.GetMaterialCount(); i++)
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
                        "Specular and NonSpecular BXDFs. The extension would be\n"
                        "fairly straightforward. In BounceSample for CameraSample\n"
                        "limit the considered events to Specular only.\n"
                        "Merging will use non-specular components, bounce will be specular.\n"
                        "If there is no specular component, the ray will terminate.\n\n");
                    printf("We are now switching from *Ppm* to *Bpm*, which can handle the scene\n\n");
                    mPpm = false;
                }
            }
        }

        mBaseRadius  = 0.004f * mScene.mSceneSphere.mSceneRadius;
        mPhotonAlpha = 0.75f;
    }

    virtual void RunIteration(int aIteration)
    {
        // While we have the same number of pixels (camera paths)
        // and light paths, we do keep them separate for clarity reasons
        const int resX = int(mScene.mCamera.mResolution.x);
        const int resY = int(mScene.mCamera.mResolution.y);
        const int pathCount = resX * resY;
        mScreenPixelCount   = float(resX * resY);
        mLightPathCount     = float(resX * resY);

        // Setup our radius, 1st iteration has aIteration == 0, thus offset
        float radius = mBaseRadius;
        radius /= std::pow(float(aIteration + 1), 0.5f * (1 - mPhotonAlpha));
        // Purely for numeric stability
        radius       = std::max(radius, 1e-7f);
        const float radiusSqr = Sqr(radius);

        // Factor used to normalise vertex merging contribution.
        // We divide the summed up energy by disk radius and number of light paths
        mVmNormalization = 1.f / (radiusSqr * PI_F * mLightPathCount);

        // set up Vm and Vc weight factors
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
                // extended by this EPS_RAY after hitpoint is determined
                Ray ray(lightSample.mOrigin + lightSample.mDirection * EPS_RAY,
                    lightSample.mDirection, 0);
                Isect isect(1e36f);

                if(!mScene.Intersect(ray, isect))
                    break;

                const Vec3f hitPoint = ray.org + ray.dir * isect.dist;
                isect.dist += EPS_RAY;

                LightBxdf bxdf(ray, isect, mScene);
                if(!bxdf.IsValid())
                    break;

                // Update MIS constants
                {
                    // Infinite lights use MIS based on solid angle instead of
                    // area, so we do not want distance there
                    if(lightSample.mPathLength > 1 || lightSample.mIsFiniteLight == 1)
                        lightSample.d0 *= Mis(Sqr(isect.dist));

                    lightSample.d0   /= Mis(std::abs(bxdf.CosThetaFix()));
                    lightSample.d1vc /= Mis(std::abs(bxdf.CosThetaFix()));
                    lightSample.d1vm /= Mis(std::abs(bxdf.CosThetaFix()));
                }

                // Store particle, purely delta bxdf cannot be merged
                if(!bxdf.IsDelta() && (mUseVC || mUseVM))
                {
                    LightVertex lightVertex;
                    lightVertex.mHitpoint   = hitPoint;
                    lightVertex.mWeight     = lightSample.mWeight;
                    lightVertex.mPathLength = lightSample.mPathLength;
                    lightVertex.mBxdf       = bxdf;

                    lightVertex.d0   = lightSample.d0;
                    lightVertex.d1vc = lightSample.d1vc;
                    lightVertex.d1vm = lightSample.d1vm;

                    mLightVertices.push_back(lightVertex);
                }

                // Contribute directly to camera, purely delta bxdf cannot be connected
                if(!bxdf.IsDelta() && (mUseVC || mLightTraceOnly) && DIR_CON)
                {
                    DirectContribution(lightSample, hitPoint, bxdf);
                }

                // We will now extend by the bounce (1) and then
                // we need 1 more segment to reach camera.
                // If that is too long, quit
                if(lightSample.mPathLength + 2 > mMaxPathLength)
                    break;

                if(!BounceSample(bxdf, hitPoint, lightSample))
                    break;
            }

            mPathEnds[pathIdx] = (int)mLightVertices.size();
        }

        //////////////////////////////////////////////////////////////////////////
        // Build hash grid
        //////////////////////////////////////////////////////////////////////////

        // Only build grid when merging (Vcm, Bpm, and Ppm)
        if(mUseVM && VM)
        {
            // The number of cells is somewhat arbitrary, but seems to work ok
            mHashGrid.Reserve(pathCount);
            mHashGrid.Build(mLightVertices, radius);
        }

        //////////////////////////////////////////////////////////////////////////
        // Generate camera paths
        //////////////////////////////////////////////////////////////////////////

        // Light tracing does not use any camera particles at all
        for(int pathIdx = 0; (pathIdx < pathCount) && (!mLightTraceOnly); pathIdx++)
        {
            PathElement cameraSample;
            const Vec2f screenSample =
                GenerateCameraSample(pathIdx, cameraSample);
            Vec3f color(0);

            //////////////////////////////////////////////////////////////////////////
            // Trace camera path
            for(;; ++cameraSample.mPathLength)
            {
                // We offset ray origin instead of setting tmin due to numeric
                // issues in ray-sphere intersection. The isect.dist has to be
                // extended by this EPS_RAY after hitpoint is determined
                Ray ray(cameraSample.mOrigin + cameraSample.mDirection * EPS_RAY,
                    cameraSample.mDirection, 0);
                Isect isect(1e36f);

                if(!mScene.Intersect(ray, isect))
                {
                    if(mScene.GetBackground() != NULL && ON_HIT)
                    {
                        color += cameraSample.mWeight *
                            LightOnHit(mScene.GetBackground(), cameraSample,
                            Vec3f(0), ray.dir);
                    }
                    break;
                }

                const Vec3f hitPoint = ray.org + ray.dir * isect.dist;
                isect.dist += EPS_RAY;

                CameraBxdf bxdf(ray, isect, mScene);
                if(!bxdf.IsValid())
                    break;

                // Update MIS constants
                {
                    cameraSample.d0   *= Mis(Sqr(isect.dist));
                    cameraSample.d0   /= Mis(std::abs(bxdf.CosThetaFix()));
                    cameraSample.d1vc /= Mis(std::abs(bxdf.CosThetaFix()));
                    cameraSample.d1vm /= Mis(std::abs(bxdf.CosThetaFix()));
                }

                // directly hit some light
                // lights do not reflect light, so we stop after this
                if(isect.lightID >= 0 && ON_HIT)
                {
                    const AbstractLight *light = mScene.GetLightPtr(isect.lightID);
                    color += cameraSample.mWeight *
                        LightOnHit(light, cameraSample, hitPoint, ray.dir);
                    break;
                }

                // Everything else needs at least one more path segment
                if(cameraSample.mPathLength >= mMaxPathLength)
                    break;

                // [Vertex Connection] Connect to lights
                if(!bxdf.IsDelta() && mUseVC && DIR_LIGHT)
                {
                    color += cameraSample.mWeight *
                        DirectIllumination(cameraSample, hitPoint, bxdf);
                }

                // [Vertex Connection] Connect to light particles
                if(!bxdf.IsDelta() && mUseVC && VC)
                {
                    // Each lightpath is assigned to one eyepath ,as in standard BPT.
                    // This gives range in which are the lightvertices
                    // corresponding to the current eye path.
                    // It is also possible to connect to vertices
                    // from any light path, but MIS should be revisited.
                    const Vec2i range(
                        (pathIdx == 0) ? 0 : mPathEnds[pathIdx-1],
                        mPathEnds[pathIdx]);

                    for(int i=range.x; i < range.y; i++)
                    {
                        const LightVertex &lightVertex = mLightVertices[i];
                        // light vertices are stored in increasing path length
                        // order, once we go above the max path length, we can
                        // skip the rest
                        if(lightVertex.mPathLength + 1 +
                            cameraSample.mPathLength > mMaxPathLength)
                            break;

                        color += cameraSample.mWeight * lightVertex.mWeight *
                            ConnectVertices(lightVertex, bxdf, hitPoint,
                            cameraSample);
                    }
                }

                // [Vertex Merging] Merge with light particles
                if(!bxdf.IsDelta() && mUseVM && VM)
                {
                    RangeQuery query(*this, hitPoint, bxdf, cameraSample);
                    mHashGrid.Process(mLightVertices, query);
                    color += cameraSample.mWeight * mVmNormalization * query.GetContrib();
                    // Ppm merges only on first non-specular bounce
                    if(mPpm)
                        break;
                }

                if(!BounceSample(bxdf, hitPoint, cameraSample))
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
    // Methods that handle camera tracing
    //////////////////////////////////////////////////////////////////////////
    /* \brief Given a pixel index, generates new camera sample
     */
    Vec2f GenerateCameraSample(
        const int    aPixelIndex,
        PathElement  &oCameraSample)
    {
        const Camera &camera    = mScene.mCamera;
        const int resX = int(camera.mResolution.x);
        const int resY = int(camera.mResolution.y);

        // Determine pixel xy
        const int x = aPixelIndex % resX;
        const int y = aPixelIndex / resX;

        // Jitter the pixel
        const Vec2f sample = Vec2f(float(x), float(y)) + mRng.GetVec2f();

        // Generate primary ray and find its pdf
        const Ray   primaryRay = camera.GenerateRay(sample);
        const float cosTheta   = Dot(camera.mForward, primaryRay.dir);
        const float cameraPdfW = 1.f / (cosTheta * cosTheta * cosTheta *
            camera.mPixelArea * mScreenPixelCount);

        oCameraSample.mOrigin       = primaryRay.org;
        oCameraSample.mDirection    = primaryRay.dir;
        oCameraSample.mWeight       = Vec3f(1);

        oCameraSample.mPathLength   = 1;
        oCameraSample.mSpecularPath = 1;

        oCameraSample.d0            = Mis(1.f / cameraPdfW);
        oCameraSample.d1vc          = 0;
        oCameraSample.d1vm          = 0;

        return sample;
    }

    /* \brief Returns (unweighted) radiance when ray hits a light source
     *
     * Can be used for both Background and Area lights.
     * For Background:
     *  Has to be called BEFORE updating MIS constants.
     *  Value of aHitpoint is irrelevant (passing Vec3f(0))
     *
     * For Area lights:
     *  Has to be called AFTER updating MIS constants.
     */
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

    // has to be called after UpdateConstantsOnHit
    Vec3f DirectIllumination(
        const PathElement  &aCameraSample,
        const Vec3f        &aHitpoint,
        const CameraBxdf   &aBxdf)
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

        float brdfDirPdfW, brdfRevPdfW, cosToLight;
        const Vec3f brdfFactor = aBxdf.EvaluateBrdfPdfW(mScene,
            directionToLight, cosToLight, &brdfDirPdfW, &brdfRevPdfW);

        if(brdfFactor.IsZero())
            return Vec3f(0);

        const float continuationProbability = aBxdf.ContinuationProb();
        // If the light is delta light, we can never hit it
        // by brdf sampling, so the probability of this path is 0
        if(light->IsDelta())
            brdfDirPdfW = 0.f;
        else
            brdfDirPdfW *= continuationProbability;

        brdfRevPdfW *= continuationProbability;

        // What we ultimately want to do is
        // ratio = emissionPdfA / directPdfA
        // What we have is:
        // emissionPdfW, and directPdfW = directPdfA * dist^2 / cosAtLight
        // Expanding we get:
        // emissionPdfA = emissionPdfW * cosToLight / dist^2
        // directPdfA   = directPdfW * cosAtLight / dist^2
        // ratio = (emissionPdfW * cosToLight / dist^2) / (directPdfW * cosAtLight / dist^2)
        // ratio = (emissionPdfW * cosToLight) / (directPdfW * cosAtLight)

        // also note, we do multiply by lightPickProb only for wLight,
        // as it does cancel out in wCamera

        // WARNING: emissionPdfW and directPdfW should be multiplied by
        // lightPickProb at this point. But to limit the numberic issues, we do NOT
        // do this, as wCamera = emissionPdf/directPdf and there the lightPickProb
        // would simply cancel out. We therefore multiply only directPdfW and only
        // where needed

        const float wLight  = Mis(brdfDirPdfW / (lightPickProb * directPdfW));
        const float wCamera = Mis(emissionPdfW * cosToLight / (directPdfW * cosAtLight)) * (
            mMisVmWeightFactor + aCameraSample.d0 + aCameraSample.d1vc * Mis(brdfRevPdfW));
        const float misWeight = 1.f / (wLight + 1.f + wCamera);

        const Vec3f contrib =
            (misWeight * cosToLight / (lightPickProb * directPdfW)) * (radiance * brdfFactor);

        if(contrib.IsZero())
            return Vec3f(0);

        if(mScene.Occluded(aHitpoint, directionToLight, distance))
            return Vec3f(0);

        return contrib;
    }

    // The direction is FROM camera TO light Vertex
    Vec3f ConnectVertices(
        const LightVertex   &aLightVertex,
        const CameraBxdf    &aCameraBxdf,
        const Vec3f         &aCameraHitpoint,
        const PathElement   &aCameraSample) const
    {
        // get the connection
        Vec3f direction   = aLightVertex.mHitpoint - aCameraHitpoint;
        const float dist2 = direction.LenSqr();
        float  distance   = std::sqrt(dist2);
        direction        /= distance;

        // evaluate brdf at camera vertex
        float cosCamera, cameraBrdfDirPdfW, cameraBrdfRevPdfW;
        const Vec3f cameraBrdfFactor = aCameraBxdf.EvaluateBrdfPdfW(
            mScene, direction, cosCamera, &cameraBrdfDirPdfW,
            &cameraBrdfRevPdfW);

        if(cameraBrdfFactor.IsZero())
            return Vec3f(0);

        // camera continuation probability (for russian roulette)
        const float cameraCont = aCameraBxdf.ContinuationProb();
        cameraBrdfDirPdfW *= cameraCont;
        cameraBrdfRevPdfW *= cameraCont;

        // evaluate brdf at light vertex
        float cosLight, lightBrdfDirPdfW, lightBrdfRevPdfW;
        const Vec3f lightBrdfFactor = aLightVertex.mBxdf.EvaluateBrdfPdfW(
            mScene, -direction, cosLight, &lightBrdfDirPdfW,
            &lightBrdfRevPdfW);

        if(lightBrdfFactor.IsZero())
            return Vec3f(0);

        // light continuation probability (for russian roulette)
        const float lightCont = aLightVertex.mBxdf.ContinuationProb();
        lightBrdfDirPdfW *= lightCont;
        lightBrdfRevPdfW *= lightCont;

        // compute geometry term
        const float geometryTerm = cosLight * cosCamera / dist2;
        if(geometryTerm < 0)
            return Vec3f(0);

        // convert pdfs to area pdf, when we have squared distance available
        const float cameraBrdfDirPdfA = PdfWtoA(cameraBrdfDirPdfW, distance, cosLight);
        const float lightBrdfDirPdfA  = PdfWtoA(lightBrdfDirPdfW,  distance, cosCamera);

        // MIS weights
        const float wLight = Mis(cameraBrdfDirPdfA) * (
            mMisVmWeightFactor + aLightVertex.d0 + aLightVertex.d1vc * Mis(lightBrdfRevPdfW));
        const float wCamera = Mis(lightBrdfDirPdfA) * (
            mMisVmWeightFactor + aCameraSample.d0 + aCameraSample.d1vc * Mis(cameraBrdfRevPdfW));

        const float misWeight = 1.f / (wLight + 1.f + wCamera);

        const Vec3f contrib = (misWeight * geometryTerm) * cameraBrdfFactor * lightBrdfFactor;
        if(contrib.IsZero())
            return Vec3f(0);

        if(mScene.Occluded(aCameraHitpoint, direction, distance))
            return Vec3f(0);

        return contrib;
    }

    //////////////////////////////////////////////////////////////////////////
    // Methods that handle light tracing
    //////////////////////////////////////////////////////////////////////////

    /* \brief Generates new light sample
     *
     * Effectively emits particle from light, storing it in oLightSample
     */
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
        oLightSample.mWeight = light->Emit(mScene.mSceneSphere, rndDirSamples, rndPosSamples,
            oLightSample.mOrigin, oLightSample.mDirection,
            emissionPdfW, &directPdfW, &cosLight);

        emissionPdfW *= lightPickProb;
        directPdfW   *= lightPickProb;

        oLightSample.mWeight       /= emissionPdfW;
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

    /* \brief Computes contribution of light sample to camera.
     *
     * It directly splats the sample into framebuffer
     */
    void DirectContribution(
        const PathElement  &aLightSample,
        const Vec3f        &aHitpoint,
        const LightBxdf    &aBxdf)
    {
        const Camera &camera    = mScene.mCamera;
        Vec3f directionToCamera = camera.mPosition - aHitpoint;
        // check point is in front of camera
        if(Dot(camera.mForward, -directionToCamera) <= 0.f)
            return;

        // check it projects to the screen (and where)
        const Vec2f imagePos = camera.WorldToRaster(aHitpoint);
        if(!camera.CheckRaster(imagePos))
            return;

        // compute distance and normalize direction to camera
        const float distEye2 = directionToCamera.LenSqr();
        const float distance            = std::sqrt(distEye2);
        directionToCamera  /= distance;

        // get the BRDF
        float cosToCamera, brdfDirPdfW, brdfRevPdfW;
        const Vec3f brdfFactor = aBxdf.EvaluateBrdfPdfW(mScene,
            directionToCamera, cosToCamera, &brdfDirPdfW, &brdfRevPdfW);
        if(brdfFactor.IsZero())
            return;

        brdfRevPdfW *= aBxdf.ContinuationProb();

        // PDF of ray from camera hitting here (w.r.t. real resolution)
        const float cosAtCamera = Dot(camera.mForward, -directionToCamera);
        const float cameraPdfW = 1.f / (cosAtCamera * cosAtCamera * cosAtCamera *
            camera.mPixelArea);
        const float cameraPdfA = PdfWtoA(cameraPdfW, distance, cosToCamera);

        // MIS weights, we need cameraPdfA w.r.t. normalised device coordinate
        // so we divide by (resolution.x * resolution.y)
        const float wLight = Mis(cameraPdfA / mScreenPixelCount) * (
            mMisVmWeightFactor + aLightSample.d0 + aLightSample.d1vc * Mis(brdfRevPdfW));

        const float misWeight = mLightTraceOnly ? 1.f :
            (1.f / (wLight + 1.f));

        const float fluxToRadianceFactor = cameraPdfA;
        const Vec3f contrib = misWeight * fluxToRadianceFactor * brdfFactor;
        if(!contrib.IsZero())
        {
            if(mScene.Occluded(aHitpoint, directionToCamera, distance))
                return;

            mFramebuffer.AddColor(imagePos,
                contrib * aLightSample.mWeight / mLightPathCount);
        }
    }


    /* \brief Bounces sample according to BXDF, returns false when terminating
     *
     * Can bounce both light and camera samples, the difference is only in Bxdf
     */
    template<bool tLightSample>
    bool BounceSample(
        const BXDF<tLightSample> &aBxdf,
        const Vec3f              &aHitPoint,
        PathElement              &aoPathSample)
    {
        Vec3f rndTriplet  = mRng.GetVec3f();
        //Vec3f rndTriplet;
        //rndTriplet.x = mRng.GetFloat();
        //rndTriplet.y = mRng.GetFloat();
        //rndTriplet.z = 0;
        float brdfDirPdfW, cosThetaOut;
        uint  sampledEvent;

        Vec3f brdfFactor = aBxdf.SampleBrdf(mScene, rndTriplet, aoPathSample.mDirection,
            brdfDirPdfW, cosThetaOut, &sampledEvent);

        if(brdfFactor.IsZero())
            return false;

        float brdfRevPdfW = aBxdf.EvaluatePdfW(mScene, aoPathSample.mDirection, true);
        // if we sampled specular event, then the reverese probability cannot be evaluated,
        // but we know it is exactly the same as direct probability, so just set it
        if(sampledEvent & LightBxdf::kSpecular)
            brdfRevPdfW = brdfDirPdfW;

        // russian roulette
        const float contProb = aBxdf.ContinuationProb();
        if(mRng.GetFloat() > contProb)
            return false;

        brdfDirPdfW *= contProb;
        brdfRevPdfW *= contProb;

        // new MIS weights
        {
            if(sampledEvent & LightBxdf::kSpecular)
            {
                aoPathSample.d0 = 0.f;
                aoPathSample.d1vc *=
                    Mis(cosThetaOut / brdfDirPdfW) * Mis(brdfRevPdfW);
                aoPathSample.d1vm *=
                    Mis(cosThetaOut / brdfDirPdfW) * Mis(brdfRevPdfW);
                aoPathSample.mSpecularPath &= 1;
            }
            else
            {
                aoPathSample.d1vc = Mis(cosThetaOut / brdfDirPdfW) * (
                    aoPathSample.d1vc * Mis(brdfRevPdfW) +
                    aoPathSample.d0 + mMisVmWeightFactor);

                aoPathSample.d1vm = Mis(cosThetaOut / brdfDirPdfW) * (
                    aoPathSample.d1vm * Mis(brdfRevPdfW) +
                    aoPathSample.d0 * mMisVcWeightFactor + 1.f);

                aoPathSample.d0 = Mis(1.f / brdfDirPdfW);

                aoPathSample.mSpecularPath &= 0;
            }
        }

        aoPathSample.mOrigin  = aHitPoint;
        aoPathSample.mWeight *= brdfFactor * (cosThetaOut / brdfDirPdfW);
        return true;
    }
private:
    bool        mUseVM;
    bool        mUseVC;
    bool        mLightTraceOnly;
    bool        mPpm;

    float       mPhotonAlpha; //!< Governs reduction rate
    float       mBaseRadius;
    float       mMisVmWeightFactor;
    float       mMisVcWeightFactor;
    float       mScreenPixelCount;
    float       mLightPathCount;
    float       mVmNormalization;


    std::vector<LightVertex> mLightVertices;
    // for lightpath belonging to pixel index [x] it stores
    // where it's light vertices end (begin is at [x-1])
    std::vector<int>         mPathEnds;
    HashGrid                 mHashGrid;

    Rng         mRng;
    //GpuQ2RandomTea mRng;
};

#endif //__VERTEXCM_HXX__
