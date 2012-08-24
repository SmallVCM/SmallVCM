/*
 * This is published under Apache 2.0
 */

#ifndef __PATHTRACER_HXX__
#define __PATHTRACER_HXX__

#include <vector>
#include <cmath>
#include <random>
#include "renderer.hxx"
#include "bxdf.hxx"
#include "rng.hxx"

class PathTracer : public AbstractRenderer
{
public:
    PathTracer(const Scene& aScene, int aSeed = 1234) : AbstractRenderer(aScene), mRng(aSeed)
    {
    }

    virtual void RunIteration(int aIteration)
    {
        // We sample lights uniformly
        const int   lightCount    = mScene.GetLightCount();
        const float lightPickProb = 1.f / lightCount;

        const int resX = int(mScene.mCamera.mResolution.x);
        const int resY = int(mScene.mCamera.mResolution.y);

        for(int pixID = 0; pixID < resX * resY; pixID++)
        {
            const int x = pixID % resX;
            const int y = pixID / resX;

            const Vec2f sample = Vec2f(float(x), float(y)) + mRng.GetVec2f();

            Ray   ray = mScene.mCamera.GenerateRay(sample);
            Isect isect;
            isect.dist = 1e36f;

            Vec3f pathWeight(1.f);
            Vec3f color(0.f);
            uint  pathLength   = 1;
            bool  lastSpecular = true;
            float lastPdfW     = 1;

            for(;; ++pathLength)
            {
                if(!mScene.Intersect(ray, isect))
                {
                    const BackgroundLight* background = mScene.GetBackground();
                    if(!background)
                        break;
                    // For background we cheat with the A/W suffixes,
                    // and GetRadiance actually returns W instead of A
                    float directPdfW;
                    Vec3f contrib = background->GetRadiance(mScene.mSceneSphere,
                        ray.dir, Vec3f(0), &directPdfW);
                    if(contrib.IsZero())
                        break;

                    float misWeight = 1.f;
                    if(pathLength > 1 && !lastSpecular)
                    {
                        misWeight = Mis2(lastPdfW, directPdfW * lightPickProb);
                    }

                    color += pathWeight * misWeight * contrib;
                    break;
                }

                Vec3f hitPoint = ray.org + ray.dir * isect.dist;

                BXDF<false> bxdf(ray, isect, mScene);
                if(!bxdf.IsValid())
                    break;

                // directly hit some light, lights do not reflect
                if(isect.lightID >= 0)
                {
                    const AbstractLight *light = mScene.GetLightPtr(isect.lightID);
                    float directPdfA;
                    Vec3f contrib = light->GetRadiance(mScene.mSceneSphere,
                        ray.dir, hitPoint, &directPdfA);
                    if(contrib.IsZero())
                        break;

                    float misWeight = 1.f;
                    if(pathLength > 1 && !lastSpecular)
                    {
                        const float directPdfW = PdfAtoW(directPdfA, isect.dist,
                            bxdf.CosThetaFix());
                        misWeight = Mis2(lastPdfW, directPdfW * lightPickProb);
                    }

                    color += pathWeight * misWeight * contrib;
                    break;
                }

                if(pathLength >= mMaxPathLength)
                    break;

                if(bxdf.ContinuationProb() == 0)
                    break;

                // next event estimation
                if(!bxdf.IsDelta())
                {
                    int lightID = int(mRng.GetFloat() * lightCount);
                    const AbstractLight *light = mScene.GetLightPtr(lightID);

                    Vec3f directionToLight;
                    float distance, directPdfW;
                    Vec3f radiance = light->Illuminate(mScene.mSceneSphere, hitPoint,
                        mRng.GetVec2f(), directionToLight, distance, directPdfW);

                    if(directPdfW > 0)
                    {
                        float brdfPdfW, cosThetaOut;
                        const Vec3f factor = bxdf.EvaluateBrdfPdfW(mScene,
                            directionToLight, cosThetaOut, &brdfPdfW);

                        if(!factor.IsZero())
                        {
                            float weight = 1.f;
                            if(!light->IsDelta())
                            {
                                const float contProb = bxdf.ContinuationProb();
                                brdfPdfW *= contProb;
                                weight = Mis2(directPdfW * lightPickProb, brdfPdfW);
                            }

                            Vec3f contrib = (weight * cosThetaOut / (lightPickProb * directPdfW)) *
                                (radiance * factor);

                            if(!mScene.Occluded(hitPoint, directionToLight, distance))
                            {
                                color += pathWeight * contrib;
                            }
                        }
                    }
                }

                // bounce
                {
                    Vec3f rndTriplet = mRng.GetVec3f();
                    float pdf, cosThetaOut;
                    uint  sampledEvent;

                    Vec3f factor = bxdf.SampleBrdf(mScene, rndTriplet, ray.dir,
                        pdf, cosThetaOut, &sampledEvent);

                    if(factor.IsZero())
                        break;

                    // russian roulette
                    const float contProb = bxdf.ContinuationProb();

                    lastSpecular = (sampledEvent & BXDF<true>::Specular) != 0;
                    lastPdfW     = pdf * contProb;

                    if(contProb < 1.f)
                    {
                        if(mRng.GetFloat() > contProb)
                        {
                            break;
                        }
                        pdf *= contProb;
                    }

                    pathWeight *= factor * (cosThetaOut / pdf);
                    // Due to numerics in sphere intersection,
                    // we actually want to offset origin from sphere surface,
                    // instead of using tmin
                    ray.org    = hitPoint + EPS_RAY * ray.dir;
                    ray.tmin   = 0.f;
                    isect.dist = 1e36f;
                }
            }
            mFramebuffer.AddColor(sample, color);
        }

        mIterations++;
    }

private:
    // Mis power (1 for balance heuristic)
    float Mis(float aPdf) const { return aPdf; }

    // Mis weight for 2 pdfs
    float Mis2(float aSamplePdf, float aOtherPdf) const
    {
        return Mis(aSamplePdf) / (Mis(aSamplePdf) + Mis(aOtherPdf));
    }

private:
    Rng         mRng;
};

#endif //__PATHTRACER_HXX__
