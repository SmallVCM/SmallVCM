/*
 * This is published under Apache 2.0
 */

#ifndef __BXDF_HXX__
#define __BXDF_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"
#include "frame.hxx"
#include "ray.hxx"
#include "scene.hxx"
#include "utils.hxx"

//////////////////////////////////////////////////////////////////////////
// BXDF, most magic happens here
//
// One of important conventions is prefixing direction with World when
// are in world coordinates and with Local when they are in local frame,
// i.e., mFrame.
//
// Another important convention if suffix Fix and Gen.
// For PDF computation, we need to know which direction is given (Fix),
// and which is the generated (Gen) direction. This is important even
// when simply evaluating BXDF.
// In BPT, we call EvaluateBrdfPdf when directly connecting to light/camera.
// This gives us both directions required for evaluating BXDF.
// However, for MIS we also need to know probabilities of having sampled
// this path via BXDF sampling, and we need that for both possible directions.
// The Fix/Gen convention (along with Direct and Reverse for PDF) clearly
// establishes which PDF is which.
//
// The BXDF is also templated by direction of tracing, whether from camera
// (BXDF<false>) or from light (BXDF<true>). This is identical to Veach's
// Adjoint BRDF (except the name is more straightforward).
// For us this is only used when refracting.


template<bool FixIsLight>
class BXDF
{
    struct ComponentProbabilities
    {
        float diffProb;
        float phongProb;
        float reflProb;
        float refrProb;
    };
public:
    enum Events
    {
        NONE        = 0,
        Diffuse     = 1,
        Phong       = 2,
        Reflect     = 4,
        Refract     = 8,
        Specular    = (Reflect | Refract),
        NonSpecular = (Diffuse | Phong),
        All         = (Specular | NonSpecular)
    };

public:
    BXDF():mMaterialID(-1){};

    BXDF(const Ray& aRay, const Isect& aIsect, const Scene& aScene)
    {
        Setup(aRay, aIsect, aScene);
    }

    void Setup(const Ray& aRay, const Isect& aIsect, const Scene& aScene)
    {
        mMaterialID = -1;
        mFrame.SetFromZ(aIsect.normal);
        mLocalOmegaFix = mFrame.ToLocal(-aRay.dir);

        // reject rays that are too parallel with tangent plane
        if(std::abs(mLocalOmegaFix.z) < 1e-6f)
        {
            return;
        }

        const Material &mat = aScene.GetMaterial(aIsect.matID);
        GetComponentProbabilities(mat, mProbabilities);

        mIsDelta = (mProbabilities.diffProb == 0) && (mProbabilities.phongProb == 0);

        // now it becomes valid
        mMaterialID = aIsect.matID;
    }

    /* \brief Given a direction, evaluates BXDF
     *
     * Returns value of BXDF, as well as cosine for the
     * aWorldOmegaGen direction.
     * Can return probability (w.r.t. solid angle W),
     * of having sampled aWorldOmegaGen given mLocalOmegaFix (oDirectPdfW),
     * and of having sampled mLocalOmegaFix given aWorldOmegaGen (oReversePdfW).
     *
     * Optionally can be limited to just some events.
     */
    Vec3f EvaluateBrdfPdfW(const Scene &aScene, const Vec3f &aWorldOmegaGen,
        float &oCosThetaGen, float *oDirectPdfW = NULL, float *oReversePdfW = NULL) const
    {
        const Vec3f localOmegaGen = mFrame.ToLocal(aWorldOmegaGen);
        oCosThetaGen = std::abs(localOmegaGen.z);

        const Material &mat = aScene.GetMaterial(mMaterialID);

        Vec3f result(0);
        result += EvaluateDiffuse(mat, localOmegaGen,
            oDirectPdfW, oReversePdfW);
        result += EvaluatePhong(mat, localOmegaGen,
            oDirectPdfW, oReversePdfW);
        return result;
    }

    /* \brief Given a direction, evaluates Pdf
     *
     * By default returns PDF with which would be aWorldOmegaGen
     * generated from mLocalOmegaFix. When aEvalRevPdf == true,
     * it provides PDF for the reverse direction.
     */
    float EvaluatePdfW(
        const Scene &aScene, const Vec3f &aWorldOmegaGen,
        const bool aEvalRevPdf = false) const
    {
        const Vec3f localOmegaGen = mFrame.ToLocal(aWorldOmegaGen);

        const Material &mat = aScene.GetMaterial(mMaterialID);

        float directPdfW  = 0;
        float reversePdfW = 0;

        EvaluatePdfWDiffuse(mat, localOmegaGen,
            &directPdfW, &reversePdfW);
        EvaluatePdfWPhong(mat, localOmegaGen,
            &directPdfW, &reversePdfW);

        return aEvalRevPdf ? reversePdfW : directPdfW;
    }

    /* \brief Given 3 random numbers, samples new direction from BXDF.
     *
     * Uses z component of random triplet to pick BXDF component from
     * which it will sample direction. If non-specular component is chosen,
     * it will also evaluate the other (non-specular) BXDF components.
     * Return BXDF factor for given direction, as well as PDF choosing that direction.
     * Can return event which has been sampled.
     * If result is Vec3f(0,0,0), then the sample should be discarded.
     */
    Vec3f SampleBrdf(const Scene &aScene, const Vec3f &aRndTriplet, Vec3f &oWorldOmegaGen,
        float &oPdfW, float &oCosThetaGen, uint *oSampledEvent = NULL) const
    {
        uint sampledEvent;
        if(aRndTriplet.z < mProbabilities.diffProb)
            sampledEvent = Diffuse;
        else if(aRndTriplet.z < mProbabilities.diffProb + mProbabilities.phongProb)
            sampledEvent = Phong;
        else if(aRndTriplet.z < mProbabilities.diffProb + mProbabilities.phongProb +
            mProbabilities.reflProb)
            sampledEvent = Reflect;
        else
            sampledEvent = Refract;

        if(oSampledEvent) *oSampledEvent = sampledEvent;

        const Material &mat = aScene.GetMaterial(mMaterialID);

        oPdfW = 0;
        Vec3f result(0);
        Vec3f localOmegaGen;

        if(sampledEvent == Diffuse)
        {
            result += SampleDiffuse(mat, aRndTriplet.GetXY(), localOmegaGen, oPdfW);
            result += EvaluatePhong(mat, localOmegaGen, &oPdfW);
        }
        else if(sampledEvent == Phong)
        {
            result += SamplePhong(mat, aRndTriplet.GetXY(), localOmegaGen, oPdfW);
            result += EvaluateDiffuse(mat, localOmegaGen, &oPdfW);
        }
        else if(sampledEvent == Reflect)
        {
            result += SampleReflect(mat, aRndTriplet.GetXY(), localOmegaGen, oPdfW);
        }
        else
        {
            result += SampleRefract(mat, aRndTriplet.GetXY(), localOmegaGen, oPdfW);
        }

        oCosThetaGen   = std::abs(localOmegaGen.z);
        if(oCosThetaGen < 1e-6f)
            return Vec3f(0.f);

        oWorldOmegaGen = mFrame.ToWorld(localOmegaGen);
        return result;
    }


    bool         IsValid()  const  { return mMaterialID >= 0; }
    bool         IsDelta()  const  { return mIsDelta;         }
    float        CosTheta() const  { return mLocalOmegaFix.z; }
    float        Albedo()   const  { return mTotalAlbedo;     }
private:
    //////////////////////////////////////////////////////////////////////////
    // Sampling methods
    // All sampling methods take material, 2 random numbers [0-1[,
    // and return BRDF factor, generated direction in local coordinates,
    // and PDF
    Vec3f SampleDiffuse(const Material &aMaterial, const Vec2f &aRndTuple,
        Vec3f &oLocalOmegaGen, float &oPdfW) const
    {
        float unweightedPdfW;
        oLocalOmegaGen = SampleCosHemisphereW(aRndTuple, &unweightedPdfW);
        oPdfW += unweightedPdfW * mProbabilities.diffProb;

        return aMaterial.mDiffuseReflectance * INV_PI_F;
    }

    Vec3f SamplePhong(const Material &aMaterial, const Vec2f &aRndTuple,
        Vec3f &oLocalOmegaGen, float &oPdfW) const
    {
        oLocalOmegaGen = SamplePowerCosHemisphereW(aRndTuple, aMaterial.mGlossiness, NULL);
        // due to numeric issues in MIS, we actually need to compute all Pdfs exactly
        // the same way all the time!!!
        const Vec3f reflLocalOmegaFixed = reflect001(mLocalOmegaFix);
        {
            Frame frame;
            frame.SetFromZ(reflLocalOmegaFixed);
            oLocalOmegaGen = frame.ToWorld(oLocalOmegaGen);
        }

        const float dot_R_Wi = Dot(reflLocalOmegaFixed, oLocalOmegaGen);
        if(dot_R_Wi <= 1e-3f)
            return Vec3f(0.f);

        EvaluatePdfWPhong(aMaterial, oLocalOmegaGen, &oPdfW);

        const Vec3f rho = aMaterial.mPhongReflectance *
            (aMaterial.mGlossiness + 2.f) * 0.5f * INV_PI_F;
        return rho * std::pow(dot_R_Wi, aMaterial.mGlossiness);
    }

    Vec3f SampleReflect(const Material &aMaterial, const Vec2f &aRndTuple,
        Vec3f &oLocalOmegaGen, float &oPdfW) const
    {
        oLocalOmegaGen = reflect001(mLocalOmegaFix);

        oPdfW += mProbabilities.reflProb;
        // BRDF is multiplied (outside) by cosine (oLocalOmegaGen.z),
        // for mirror this shouldn't be done, so we pre-divide here instead
        return aMaterial.mMirrorReflectance / std::abs(oLocalOmegaGen.z);
    }

    Vec3f SampleRefract(const Material &aMaterial, const Vec2f &aRndTuple,
        Vec3f &oLocalOmegaGen, float &oPdfW) const
    {
        if(aMaterial.mIOR < 0)
            return Vec3f(0);

        float cosI = mLocalOmegaFix.z;

        float cosT;
        float etaIncOverEtaTrans;

        if(cosI < 0.f) // hit from inside
        {
            etaIncOverEtaTrans = aMaterial.mIOR;
            cosI = -cosI;
            cosT = 1.f;
        }
        else
        {
            etaIncOverEtaTrans = 1.f / aMaterial.mIOR;
            cosT = -1.f;
        }

        const float sinI2 = 1.f - cosI * cosI;
        const float sinT2 = Sqr(etaIncOverEtaTrans) * sinI2;

        if(sinT2 < 1.f) // no total internal reflection
        {
            cosT *= std::sqrt(std::max(0.f, 1.f - sinT2));

            oLocalOmegaGen = Vec3f(
                -etaIncOverEtaTrans * mLocalOmegaFix.x,
                -etaIncOverEtaTrans * mLocalOmegaFix.y,
                cosT);

            oPdfW += mProbabilities.refrProb;

            // only camera paths are multiplied by this factor, and etas
            // are swapped because radiance flows in the opposite direction
            if(!FixIsLight)
                return Vec3f(Sqr(etaIncOverEtaTrans) / std::abs(cosT));
            else
                return Vec3f(1.f / std::abs(cosT));
        }
        //else total internal reflection, do nothing
        oPdfW += 0.f;
        return Vec3f(0.f);
    }

    //////////////////////////////////////////////////////////////////////////
    // Evaluating methods
    Vec3f EvaluateDiffuse(const Material& aMaterial, const Vec3f& aLocalOmegaGen,
        float *oDirectPdfW = NULL, float *oReversePdfW = NULL) const
    {
        if(mProbabilities.diffProb == 0) return Vec3f(0);

        if(oDirectPdfW)
            *oDirectPdfW  += mProbabilities.diffProb *
            std::max(0.f, aLocalOmegaGen.z * INV_PI_F);

        if(oReversePdfW)
            *oReversePdfW += mProbabilities.diffProb *
            std::max(0.f, mLocalOmegaFix.z * INV_PI_F);

        return aMaterial.mDiffuseReflectance * INV_PI_F;
    }

    Vec3f EvaluatePhong(const Material& aMaterial, const Vec3f& aLocalOmegaGen,
        float *oDirectPdfW = NULL, float *oReversePdfW = NULL) const
    {
        if(mProbabilities.phongProb == 0) return Vec3f(0);

        // assumes this is never called when rejectShadingCos(oLocalOmegaGen.z) is true
        const Vec3f reflLocalOmegaIn = reflect001(mLocalOmegaFix);
        const float dot_R_Wi = Dot(reflLocalOmegaIn, aLocalOmegaGen);

        if(dot_R_Wi <= 1e-3f)
            return Vec3f(0.f);

        if(oDirectPdfW || oReversePdfW)
        {
            // the sampling is symmetric
            const float pdfW = EvalPowerCosHemispherePdfW(reflLocalOmegaIn, aLocalOmegaGen,
                aMaterial.mGlossiness) * mProbabilities.phongProb;
            if(oDirectPdfW)  *oDirectPdfW  += pdfW;
            if(oReversePdfW) *oReversePdfW += pdfW;
        }

        const Vec3f rho = aMaterial.mPhongReflectance *
            (aMaterial.mGlossiness + 2.f) * 0.5f * INV_PI_F;
        return rho * std::pow(dot_R_Wi, aMaterial.mGlossiness);
    }

    //////////////////////////////////////////////////////////////////////////
    // Pdf rvaluating methods
    void EvaluatePdfWDiffuse(const Material& aMaterial, const Vec3f& aLocalOmegaGen,
        float *oDirectPdfW = NULL, float *oReversePdfW = NULL) const
    {
        if(mProbabilities.diffProb == 0) return Vec3f(0);

        if(oDirectPdfW)
            *oDirectPdfW  += mProbabilities.diffProb *
            std::max(0, aLocalOmegaGen.z * INV_PI_F);

        if(oReversePdfW)
            *oReversePdfW += mProbabilities.diffProb *
            std::max(0, mLocalOmegaFix.z * INV_PI_F);
    }

    void EvaluatePdfWPhong(const Material& aMaterial, const Vec3f& aLocalOmegaGen,
        float *oDirectPdfW = NULL, float *oReversePdfW = NULL) const
    {
        if(mProbabilities.phongProb == 0) return;

        // assumes this is never called when rejectShadingCos(oLocalOmegaGen.z) is true
        const Vec3f reflLocalOmegaIn = reflect001(mLocalOmegaFix);
        const float dot_R_Wi = Dot(reflLocalOmegaIn, aLocalOmegaGen);

        if(dot_R_Wi <= 1e-3f)
            return;

        if(oDirectPdfW || oReversePdfW)
        {
            // the sampling is symmetric
            const float pdfW = EvalPowerCosHemispherePdfW(reflLocalOmegaIn, aLocalOmegaGen,
                aMaterial.mGlossiness) * mProbabilities.phongProb;
            if(oDirectPdfW)  *oDirectPdfW  += pdfW;
            if(oReversePdfW) *oReversePdfW += pdfW;
        }
    }

    //////////////////////////////////////////////////////////////////////////
    // Albedo methods
    float AlbedoDiffuse(const Material& aMaterial) const
    {
        return Luminance(aMaterial.mDiffuseReflectance);
    }

    float AlbedoPhong(const Material& aMaterial) const
    {
        return Luminance(aMaterial.mPhongReflectance);
    }

    float AlbedoReflect(const Material& aMaterial) const
    {
        return Luminance(aMaterial.mMirrorReflectance);
    }

    float AlbedoRefract(const Material& aMaterial) const
    {
        return Luminance(aMaterial.mGlassTransmittance);
    }

    // returns false when the material is completely black
    void GetComponentProbabilities(const Material& aMaterial,
        ComponentProbabilities& oProbabilities)
    {
        const float mirrorCoeff = FresnelDielectric(mLocalOmegaFix.z, aMaterial.mIOR);

        const float albedoDiffuse = AlbedoDiffuse(aMaterial);
        const float albedoPhong   = AlbedoPhong(aMaterial);
        const float albedoReflect = mirrorCoeff         * AlbedoReflect(aMaterial);
        const float albedoRefract = (1.f - mirrorCoeff) * AlbedoRefract(aMaterial);

        const float totalAlbedo = albedoDiffuse + albedoPhong + albedoReflect + albedoRefract;

        if(totalAlbedo < 1e-9f)
        {
            oProbabilities.diffProb  = 0.f;
            oProbabilities.phongProb = 0.f;
            oProbabilities.reflProb  = 0.f;
            oProbabilities.refrProb  = 0.f;
            mTotalAlbedo = 0.f;
        }
        else
        {
            oProbabilities.diffProb  = albedoDiffuse / totalAlbedo;
            oProbabilities.phongProb = albedoPhong   / totalAlbedo;
            oProbabilities.reflProb  = albedoReflect / totalAlbedo;
            oProbabilities.refrProb  = albedoRefract / totalAlbedo;
            mTotalAlbedo = totalAlbedo;
        }
    }

private:
    int   mMaterialID;
    Frame mFrame;
    Vec3f mLocalOmegaFix;
    bool  mIsDelta;
    ComponentProbabilities mProbabilities;
    float mTotalAlbedo;
};

#endif //__BXDF_HXX__
