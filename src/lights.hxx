/*
 * This is published under Apache 2.0
 */

#ifndef __LIGHTS_HXX__
#define __LIGHTS_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"

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
    virtual Vec3f Illuminate(const Vec3f& aReceivingPosition,
        const Vec2f& aRndTuple, Vec3f& oDirectionToLight, float& oDistance,
        float& oDirectPdfW, float* oEmissionPdfW = NULL, float* oCosAtLight = NULL) const = 0;

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
    virtual Vec3f Emit(const Vec2f &aDirRndTuple, const Vec2f &aPosRndTuple,
        Vec3f &oPosition, Vec3f &oDirection, float &oEmissionPdfW,
        float *oDirectPdfA, float *oCosThetaLight) const = 0;

    /* \brief Returns radiance for ray randomly hitting the light
     *
     * Given ray direction and hitpoint, it returns radiance.
     * Can also provide area pdf of sampling hitpoint in Illuminate,
     * and of emitting particle along the ray (in opposite direction).
     */
    virtual Vec3f GetRadiance(const Vec3f &aRayDirection,
        const Vec3f &aHitPoint, float *oDirectPdfA = NULL, float *oEmissionPdfW = NULL) const = 0;

    // Whether the light has a finite extent (area, point) or not (directional, env. map)
    virtual bool IsFinite() const = 0;
    // Whether the light has delta function (point, directional) or not (area)
    virtual bool IsDelta() const = 0;
};

class AreaLight : public AbstractLight
{
public:
    AreaLight(){}
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
    virtual Vec3f Illuminate(const Vec3f& aReceivingPosition,
        const Vec2f& aRndTuple, Vec3f& oDirectionToLight, float& oDistance,
        float& oDirectPdfW, float* oEmissionPdfW = NULL, float* oCosAtLight = NULL) const
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
    virtual Vec3f Emit(const Vec2f &aDirRndTuple, const Vec2f &aPosRndTuple,
        Vec3f &oPosition, Vec3f &oDirection, float &oEmissionPdfW,
        float *oDirectPdfA, float *oCosThetaLight) const
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

    /* \brief Returns radiance for ray randomly hitting the light
     *
     * Given ray direction and hitpoint, it returns radiance.
     * Can also provide area pdf of sampling hitpoint in Illuminate,
     * and of emitting particle along the ray (in opposite direction).
     */
    virtual Vec3f GetRadiance(const Vec3f &aRayDirection,
        const Vec3f &aHitPoint, float *oDirectPdfA = NULL, float *oEmissionPdfW = NULL) const
    {
        const float cosOutL = std::max(0.f, Dot(mFrame.Normal(), -aRayDirection));

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

#endif //__LIGHTS_HXX__
