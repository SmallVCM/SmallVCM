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
    PathTracer(Vec2f mResolution, int aSeed = 1234) : mRng(aSeed)
    {
        mIterations = 0;
        mFramebuffer.Setup(mResolution);
        mMaxPathLength = 10;
    }

    virtual void RunIteration(int aIteration, const Scene& aScene)
    {
        // We sample lights uniformly
        const int   lightCount    = aScene.GetLightCount();
        const float lightPickProb = 1.f / lightCount;

        const int resX = int(aScene.mCamera.mResolution.x);
        const int resY = int(aScene.mCamera.mResolution.y);

        for(int pixID = 0; pixID < resX * resY; pixID++)
        {
            const int x = pixID % resX;
            const int y = pixID / resX;

            const Vec2f sample = Vec2f(float(x), float(y)) + mRng.GetVec2f();

            Ray   ray = aScene.mCamera.GenerateRay(sample);
            Isect isect;
            isect.dist = 1e36f;

            Vec3f pathWeight(1.f);
            Vec3f color(0.f);
            int   pathLength   = 1;
            bool  lastSpecular = true;
            float lastPdfW     = 1;

            for(;; ++pathLength)
            {
                if(!aScene.Intersect(ray, isect))
                    break;

                Vec3f hitPoint = ray.org + ray.dir * isect.dist;

                BXDF<false> bxdf(ray, isect, aScene);
                if(!bxdf.IsValid())
                    break;

                // directly hit some light, lights do not reflect
                if(isect.lightID >= 0)
                {
                    const AbstractLight *light = aScene.GetLightPtr(isect.lightID);
                    float directPdfA;
                    Vec3f contrib = light->GetRadiance(ray.dir, hitPoint, &directPdfA);
                    if(contrib.IsZero())
                        break;

                    float misWeight = 1.f;
                    if(pathLength > 1 && !lastSpecular)
                    {
                        const float directPdfW = PdfAtoW(directPdfA, isect.dist,
                            bxdf.CosTheta());
                        misWeight = Mis2(lastPdfW, directPdfW * lightPickProb);
                    }

                    color += pathWeight * misWeight * contrib;
                    break;
                }

                if(pathLength >= mMaxPathLength)
                    break;

                if(bxdf.Albedo() == 0)
                    break;

                // next event estimation
                if(!bxdf.IsDelta())
                {
                    int lightID = mRng.GetInt() % lightCount;
                    const AbstractLight *light = aScene.GetLightPtr(lightID);

                    Vec3f directionToLight;
                    float distance, directPdfW;
                    Vec3f radiance = light->Illuminate(hitPoint,
                        mRng.GetVec2f(), directionToLight, distance, directPdfW);

                    if(directPdfW > 0)
                    {
                        float brdfPdfW, cosThetaOut;
                        const Vec3f factor = bxdf.EvaluateBrdfPdfW(aScene,
                            directionToLight, cosThetaOut, &brdfPdfW);

                        if(!factor.IsZero())
                        {
                            float weight = 1.f;
                            if(!light->IsDelta())
                            {
                                const float contProb = std::min(1.f, bxdf.Albedo());
                                brdfPdfW *= contProb;
                                weight = Mis2(directPdfW * lightPickProb, brdfPdfW);
                            }

                            Vec3f contrib = (weight * cosThetaOut / (lightPickProb * directPdfW)) *
                                (radiance * factor);

                            if(!aScene.Occluded(hitPoint, directionToLight, distance))
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

                    Vec3f factor = bxdf.SampleBrdf(aScene, rndTriplet, ray.dir,
                        pdf, cosThetaOut, &sampledEvent);

                    if(factor.IsZero())
                        break;

                    // russian roulette
                    const float contProb = std::min(1.f, bxdf.Albedo());

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

    virtual void GetFramebuffer(Framebuffer& oFramebuffer)
    {
        oFramebuffer = mFramebuffer;
        if(mIterations > 0)
            oFramebuffer.Scale(1.f / mIterations);
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
    int         mIterations;
    Framebuffer mFramebuffer;
    int         mMaxPathLength;
    Rng         mRng;
};

#endif //__PATHTRACER_HXX__
