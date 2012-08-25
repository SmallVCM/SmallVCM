/*
 * This is published under Apache 2.0
 */

#ifndef __LIGHTS_HXX__
#define __LIGHTS_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"

struct SceneSphere
{
    // Center of the scene's bounding sphere
    Vec3f mSceneCenter;
    // Radius of the scene's bounding sphere
    float mSceneRadius;
    // 1.f / (mSceneRadius^2)
    float mInvSceneRadiusSqr;
};

class AbstractLight
{
public:
    /* \brief Illuminates a given point in the scene.
     *
     * Given a point and two random samples (e.g., for position on area lights),
     * this method returns direction from point to light, distance,
     * pdf of having chosen this direction (e.g., 1 / area).
     * Optionally also returns pdf of emitting particle in this direction,
     * and cosine from lights normal (helps with PDF of hitting the light,
     * but set to 1 for point lights).
     *
     * Returns radiance.
     */
    virtual Vec3f Illuminate(
        const SceneSphere &aSceneSphere,
        const Vec3f       &aReceivingPosition,
        const Vec2f       &aRndTuple,
        Vec3f             &oDirectionToLight,
        float             &oDistance,
        float             &oDirectPdfW,
        float             *oEmissionPdfW = NULL,
        float             *oCosAtLight = NULL) const = 0;

    /* \brief Emits particle from the light.
     *
     * Given two sets of random numbers (e.g., position and direction on area light),
     * this method generates a position and direction for light particle, along
     * with the pdf.
     *
     * Can also supply pdf (w.r.t. area) of choosing this position when calling
     * Illuminate. Also provides cosine on the light (this is 1 for point lights etc.).
     *
     * Returns "energy" that particle carries
     */
    virtual Vec3f Emit(
        const SceneSphere &aSceneSphere,
        const Vec2f       &aDirRndTuple,
        const Vec2f       &aPosRndTuple,
        Vec3f             &oPosition,
        Vec3f             &oDirection,
        float             &oEmissionPdfW,
        float             *oDirectPdfA,
        float             *oCosThetaLight) const = 0;

    /* \brief Returns radiance for ray randomly hitting the light
     *
     * Given ray direction and hitpoint, it returns radiance.
     * Can also provide area pdf of sampling hitpoint in Illuminate,
     * and of emitting particle along the ray (in opposite direction).
     */
    virtual Vec3f GetRadiance(
        const SceneSphere &aSceneSphere,
        const Vec3f       &aRayDirection,
        const Vec3f       &aHitPoint,
        float             *oDirectPdfA = NULL,
        float             *oEmissionPdfW = NULL) const = 0;

    // Whether the light has a finite extent (area, point) or not (directional, env. map)
    virtual bool IsFinite() const = 0;
    // Whether the light has delta function (point, directional) or not (area)
    virtual bool IsDelta() const = 0;
};

//////////////////////////////////////////////////////////////////////////
class AreaLight : public AbstractLight
{
public:
    AreaLight(const Vec3f& aP0, const Vec3f& aP1, const Vec3f& aP2)
    {
        p0 = aP0;
        e1 = aP1 - aP0;
        e2 = aP2 - aP0;

        Vec3f normal = Cross(e1, e2);
        float len    = normal.Length();
        mInvArea     = 2.f / len;
        mFrame.SetFromZ(normal);
    }

    virtual Vec3f Illuminate(
        const SceneSphere &/*aSceneSphere*/,
        const Vec3f       &aReceivingPosition,
        const Vec2f       &aRndTuple,
        Vec3f             &oDirectionToLight,
        float             &oDistance,
        float             &oDirectPdfW,
        float             *oEmissionPdfW = NULL,
        float             *oCosAtLight = NULL) const
    {
        const Vec2f uv = SampleUniformTriangle(aRndTuple);
        const Vec3f lightPoint = p0 + e1 * uv.x + e2 * uv.y;

        oDirectionToLight     = lightPoint - aReceivingPosition;
        const float distSqr   = oDirectionToLight.LenSqr();
        oDistance             = std::sqrt(distSqr);
        oDirectionToLight     = oDirectionToLight / oDistance;

        const float cosNormalDir = Dot(mFrame.Normal(), -oDirectionToLight);

        // too close to, or under, tangent
        if(cosNormalDir < EPS_COSINE)
        {
            oDirectPdfW  = -1.f;
            return Vec3f(0.f);
        }

        oDirectPdfW = mInvArea * distSqr / cosNormalDir;

        if(oCosAtLight) *oCosAtLight = cosNormalDir;
        if(oEmissionPdfW)
        {
            *oEmissionPdfW = mInvArea * cosNormalDir * INV_PI_F;
        }
        return mIntensity;
    }

    virtual Vec3f Emit(
        const SceneSphere &/*aSceneSphere*/,
        const Vec2f       &aDirRndTuple,
        const Vec2f       &aPosRndTuple,
        Vec3f             &oPosition,
        Vec3f             &oDirection,
        float             &oEmissionPdfW,
        float             *oDirectPdfA,
        float             *oCosThetaLight) const
    {
        const Vec2f uv = SampleUniformTriangle(aPosRndTuple);
        oPosition = p0 + e1 * uv.x + e2 * uv.y;

        Vec3f localOmegaOut;
        localOmegaOut =
            SampleCosHemisphereW(aDirRndTuple, &oEmissionPdfW);

        oEmissionPdfW *= mInvArea;

        // cannot really not emit the particle, so just bias it to the correct angle
        localOmegaOut.z = std::max(localOmegaOut.z, EPS_COSINE);
        oDirection      = mFrame.ToWorld(localOmegaOut);

        if(oDirectPdfA)    *oDirectPdfA    = mInvArea;
        if(oCosThetaLight) *oCosThetaLight = localOmegaOut.z;

        return mIntensity * localOmegaOut.z;
    }

    virtual Vec3f GetRadiance(
        const SceneSphere &/*aSceneSphere*/,
        const Vec3f       &aRayDirection,
        const Vec3f       &aHitPoint,
        float             *oDirectPdfA = NULL,
        float             *oEmissionPdfW = NULL) const
    {
        const float cosOutL = std::max(0.f, Dot(mFrame.Normal(), -aRayDirection));
        if(cosOutL == 0)
            return Vec3f(0);

        if(oDirectPdfA) *oDirectPdfA = mInvArea;

        if(oEmissionPdfW)
        {
            *oEmissionPdfW =
                EvalCosHemispherePdfW(mFrame.Normal(), -aRayDirection);
            *oEmissionPdfW *= mInvArea;
        }

        return mIntensity;
    }
    // Whether the light has a finite extent (area, point) or not (directional, env. map)
    virtual bool IsFinite() const { return true; };
    // Whether the light has delta function (point, directional) or not (area)
    virtual bool IsDelta() const { return false; };

public:
    Vec3f p0, e1, e2;
    Frame mFrame;
    Vec3f mIntensity;
    float mInvArea;
};

//////////////////////////////////////////////////////////////////////////
class DirectionalLight : public AbstractLight
{
public:
    DirectionalLight(const Vec3f& aDirection)
    {
        mFrame.SetFromZ(aDirection);
    }

    virtual Vec3f Illuminate(
        const SceneSphere &aSceneSphere,
        const Vec3f       &/*aReceivingPosition*/,
        const Vec2f       &/*aRndTuple*/,
        Vec3f             &oDirectionToLight,
        float             &oDistance,
        float             &oDirectPdfW,
        float             *oEmissionPdfW = NULL,
        float             *oCosAtLight = NULL) const
    {
        oDirectionToLight     = -mFrame.Normal();
        oDistance             = 1e36f;
        oDirectPdfW           = 1.f;

        if(oCosAtLight) *oCosAtLight = 1.f;
        if(oEmissionPdfW)
        {
            *oEmissionPdfW = EvalConcentricDiscPdfA() * aSceneSphere.mInvSceneRadiusSqr;
        }
        return mIntensity;
    }

    virtual Vec3f Emit(
        const SceneSphere &aSceneSphere,
        const Vec2f       &/*aDirRndTuple*/,
        const Vec2f       &aPosRndTuple,
        Vec3f             &oPosition,
        Vec3f             &oDirection,
        float             &oEmissionPdfW,
        float             *oDirectPdfA,
        float             *oCosThetaLight) const
    {
        const Vec2f xy = SampleConcentricDisc(aPosRndTuple);

        oPosition = aSceneSphere.mSceneCenter +
            aSceneSphere.mSceneRadius * (
            -mFrame.Normal() + mFrame.Binormal() * xy.x + mFrame.Tangent() * xy.y);

        oDirection = mFrame.Normal();
        oEmissionPdfW = EvalConcentricDiscPdfA() * aSceneSphere.mInvSceneRadiusSqr;

        if(oDirectPdfA)    *oDirectPdfA = 1.f;
        // This is not used for infinite or delta lights
        if(oCosThetaLight) *oCosThetaLight = 1.f;

        return mIntensity;
    }

    virtual Vec3f GetRadiance(
        const SceneSphere &/*aSceneSphere*/,
        const Vec3f       &/*aRayDirection*/,
        const Vec3f       &/*aHitPoint*/,
        float             *oDirectPdfA = NULL,
        float             *oEmissionPdfW = NULL) const
    {
        return Vec3f(0);
    }
    // Whether the light has a finite extent (area, point) or not (directional, env. map)
    virtual bool IsFinite() const { return false; };
    // Whether the light has delta function (point, directional) or not (area)
    virtual bool IsDelta() const  { return true; };

public:
    Frame mFrame;
    Vec3f mIntensity;
};


//////////////////////////////////////////////////////////////////////////
class PointLight : public AbstractLight
{
public:
    PointLight(const Vec3f& aPosition)
    {
        mPosition = aPosition;
    }

    virtual Vec3f Illuminate(
        const SceneSphere &/*aSceneSphere*/,
        const Vec3f       &aReceivingPosition,
        const Vec2f       &aRndTuple,
        Vec3f             &oDirectionToLight,
        float             &oDistance,
        float             &oDirectPdfW,
        float             *oEmissionPdfW = NULL,
        float             *oCosAtLight = NULL) const
    {
        oDirectionToLight     = mPosition - aReceivingPosition;
        const float distSqr   = oDirectionToLight.LenSqr();
        oDirectPdfW           = distSqr;
        oDistance             = std::sqrt(distSqr);
        oDirectionToLight     = oDirectionToLight / oDistance;

        if(oCosAtLight) *oCosAtLight = 1.f;
        if(oEmissionPdfW)
        {
            *oEmissionPdfW = EvalUniformSpherePdfW();
        }
        return mIntensity;
    }

    virtual Vec3f Emit(
        const SceneSphere &/*aSceneSphere*/,
        const Vec2f       &aDirRndTuple,
        const Vec2f       &/*aPosRndTuple*/,
        Vec3f             &oPosition,
        Vec3f             &oDirection,
        float             &oEmissionPdfW,
        float             *oDirectPdfA,
        float             *oCosThetaLight) const
    {
        oPosition  = mPosition;
        oDirection = SampleUniformSphereW(aDirRndTuple, &oEmissionPdfW);

        if(oDirectPdfA)    *oDirectPdfA = 1.f;
        // This is not used for infinite or delta lights
        if(oCosThetaLight) *oCosThetaLight = 1.f;

        return mIntensity;
    }

    virtual Vec3f GetRadiance(
        const SceneSphere &/*aSceneSphere*/,
        const Vec3f       &/*aRayDirection*/,
        const Vec3f       &/*aHitPoint*/,
        float             *oDirectPdfA = NULL,
        float             *oEmissionPdfW = NULL) const
    {
        return Vec3f(0);
    }
    // Whether the light has a finite extent (area, point) or not (directional, env. map)
    virtual bool IsFinite() const { return true;  };
    // Whether the light has delta function (point, directional) or not (area)
    virtual bool IsDelta() const  { return false; };

public:
    Vec3f mPosition;
    Vec3f mIntensity;
};


//////////////////////////////////////////////////////////////////////////
class BackgroundLight : public AbstractLight
{
public:
    BackgroundLight()
    {
        mBackgroundColor = Vec3f(135, 206, 250) / Vec3f(255.f);
        mScale = 1.f;
    }

    virtual Vec3f Illuminate(
        const SceneSphere &aSceneSphere,
        const Vec3f       &aReceivingPosition,
        const Vec2f       &aRndTuple,
        Vec3f             &oDirectionToLight,
        float             &oDistance,
        float             &oDirectPdfW,
        float             *oEmissionPdfW = NULL,
        float             *oCosAtLight = NULL) const
    {
        // Replace these two lines with image sampling
        oDirectionToLight = SampleUniformSphereW(aRndTuple, &oDirectPdfW);
        //oDirectionToLight = Vec3f(0.16123600f, -0.98195398f, 0.098840252f);
        Vec3f radiance = mBackgroundColor * mScale;

        // This stays even with image sampling
        oDistance = 1e36f;
        if(oEmissionPdfW)
            *oEmissionPdfW = oDirectPdfW *
            EvalConcentricDiscPdfA() * aSceneSphere.mInvSceneRadiusSqr;
        if(oCosAtLight) *oCosAtLight = 1.f;

        return radiance;
    }

    virtual Vec3f Emit(
        const SceneSphere &aSceneSphere,
        const Vec2f       &aDirRndTuple,
        const Vec2f       &aPosRndTuple,
        Vec3f             &oPosition,
        Vec3f             &oDirection,
        float             &oEmissionPdfW,
        float             *oDirectPdfA,
        float             *oCosThetaLight) const
    {
        float directPdf;
        // Replace these two lines with image sampling
        oDirection = SampleUniformSphereW(aDirRndTuple, &directPdf);
        //oDirection = -Vec3f(0.16123600f, -0.98195398f, 0.098840252f);
        Vec3f radiance = mBackgroundColor * mScale;

        // This stays even with image sampling
        const Vec2f xy = SampleConcentricDisc(aPosRndTuple);

        Frame frame;
        frame.SetFromZ(oDirection);
        oPosition = aSceneSphere.mSceneCenter +
            aSceneSphere.mSceneRadius * (
            -oDirection + frame.Binormal() * xy.x + frame.Tangent() * xy.y);
        //oPosition = Vec3f(-1.109054f, -2.15064538f, -1.087019148f);

        oEmissionPdfW = directPdf * EvalConcentricDiscPdfA() *
            aSceneSphere.mInvSceneRadiusSqr;

        // for background we lie about Pdf being in area measure
        if(oDirectPdfA)    *oDirectPdfA    = directPdf;
        // This is not used for infinite or delta lights
        if(oCosThetaLight) *oCosThetaLight = 1.f;

        return radiance;
    }

    virtual Vec3f GetRadiance(
        const SceneSphere &aSceneSphere,
        const Vec3f       &/*aRayDirection*/,
        const Vec3f       &/*aHitPoint*/,
        float             *oDirectPdfA = NULL,
        float             *oEmissionPdfW = NULL) const
    {
        // Replace this with image lookup (proper pdf and such)
        // use aRayDirection
        float directPdf = EvalUniformSpherePdfW();
        Vec3f radiance  = mBackgroundColor * mScale;

        const float positionPdf = EvalConcentricDiscPdfA() *
            aSceneSphere.mInvSceneRadiusSqr;

        if(oDirectPdfA)   *oDirectPdfA   = directPdf;
        if(oEmissionPdfW) *oEmissionPdfW = directPdf * positionPdf;
        return radiance;
    }
    // Whether the light has a finite extent (area, point) or not (directional, env. map)
    virtual bool IsFinite() const { return false; };
    // Whether the light has delta function (point, directional) or not (area)
    virtual bool IsDelta() const  { return false; };
public:
    Vec3f mBackgroundColor;
    float mScale;
};
#endif //__LIGHTS_HXX__
