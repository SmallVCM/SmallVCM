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

#ifndef __MATH_HXX__
#define __MATH_HXX__

#include <cmath>
// for portability issues
#define PI_F     3.14159265358979f
#define INV_PI_F (1.f / PI_F)

template<typename T>
T Sqr(const T& a) { return a*a; }

typedef unsigned uint;

//////////////////////////////////////////////////////////////////////////
// Math section
template<typename T>
class Vec2x
{
public:

    Vec2x(){}
    Vec2x(T a):x(a),y(a){}
    Vec2x(T a, T b):x(a),y(b){}

    const T& Get(int a) const { return reinterpret_cast<const T*>(this)[a]; }
    T&       Get(int a)       { return reinterpret_cast<T*>(this)[a]; }

    // unary minus
    Vec2x<T> operator-() const
    { Vec2x<T> res; for(int i=0; i<2; i++) res.Get(i) = -Get(i); return res; }

    // binary operations
    friend Vec2x<T> operator+(const Vec2x& a, const Vec2x& b)
    { Vec2x<T> res; for(int i=0; i<2; i++) res.Get(i) = a.Get(i) + b.Get(i); return res; }
    friend Vec2x<T> operator-(const Vec2x& a, const Vec2x& b)
    { Vec2x<T> res; for(int i=0; i<2; i++) res.Get(i) = a.Get(i) - b.Get(i); return res; }
    friend Vec2x<T> operator*(const Vec2x& a, const Vec2x& b)
    { Vec2x<T> res; for(int i=0; i<2; i++) res.Get(i) = a.Get(i) * b.Get(i); return res; }
    friend Vec2x<T> operator/(const Vec2x& a, const Vec2x& b)
    { Vec2x<T> res; for(int i=0; i<2; i++) res.Get(i) = a.Get(i) / b.Get(i); return res; }

    Vec2x<T>& operator+=(const Vec2x& a)
    { for(int i=0; i<2; i++) Get(i) += a.Get(i); return *this;}
    Vec2x<T>& operator-=(const Vec2x& a)
    { for(int i=0; i<2; i++) Get(i) -= a.Get(i); return *this;}
    Vec2x<T>& operator*=(const Vec2x& a)
    { for(int i=0; i<2; i++) Get(i) *= a.Get(i); return *this;}
    Vec2x<T>& operator/=(const Vec2x& a)
    { for(int i=0; i<2; i++) Get(i) /= a.Get(i); return *this;}

    friend T Dot(const Vec2x& a, const Vec2x& b)
    { T res(0); for(int i=0; i<2; i++) res += a.Get(i) * b.Get(i); return res; }

public:

    T x, y;
};

typedef Vec2x<float> Vec2f;
typedef Vec2x<int>   Vec2i;

template<typename T>
class Vec3x
{
public:

    Vec3x(){}
    Vec3x(T a):x(a),y(a),z(a){}
    Vec3x(T a, T b, T c):x(a),y(b),z(c){}

    const T& Get(int a) const { return reinterpret_cast<const T*>(this)[a]; }
    T&       Get(int a)       { return reinterpret_cast<T*>(this)[a]; }
    Vec2x<T> GetXY() const    { return Vec2x<T>(x, y); }
    T        Max()   const    { T res = Get(0); for(int i=1; i<3; i++) res = std::max(res, Get(i)); return res;}
    
    bool     IsZero() const
    {
        for(int i=0; i<3; i++)
            if(Get(i) != 0)
                return false;
        return true;
    }

    // unary minus
    Vec3x<T> operator-() const
    { Vec3x<T> res; for(int i=0; i<3; i++) res.Get(i) = -Get(i); return res; }

    // binary operations
    friend Vec3x<T> operator+(const Vec3x& a, const Vec3x& b)
    { Vec3x<T> res; for(int i=0; i<3; i++) res.Get(i) = a.Get(i) + b.Get(i); return res; }
    friend Vec3x<T> operator-(const Vec3x& a, const Vec3x& b)
    { Vec3x<T> res; for(int i=0; i<3; i++) res.Get(i) = a.Get(i) - b.Get(i); return res; }
    friend Vec3x<T> operator*(const Vec3x& a, const Vec3x& b)
    { Vec3x<T> res; for(int i=0; i<3; i++) res.Get(i) = a.Get(i) * b.Get(i); return res; }
    friend Vec3x<T> operator/(const Vec3x& a, const Vec3x& b)
    { Vec3x<T> res; for(int i=0; i<3; i++) res.Get(i) = a.Get(i) / b.Get(i); return res; }

    Vec3x<T>& operator+=(const Vec3x& a)
    { for(int i=0; i<3; i++) Get(i) += a.Get(i); return *this;}
    Vec3x<T>& operator-=(const Vec3x& a)
    { for(int i=0; i<3; i++) Get(i) -= a.Get(i); return *this;}
    Vec3x<T>& operator*=(const Vec3x& a)
    { for(int i=0; i<3; i++) Get(i) *= a.Get(i); return *this;}
    Vec3x<T>& operator/=(const Vec3x& a)
    { for(int i=0; i<3; i++) Get(i) /= a.Get(i); return *this;}

    friend T Dot(const Vec3x& a, const Vec3x& b)
    { T res(0); for(int i=0; i<3; i++) res += a.Get(i) * b.Get(i); return res; }

    float    LenSqr() const   { return Dot(*this, *this);   }
    float    Length() const   { return std::sqrt(LenSqr()); }

public:

    T x, y, z;
};

typedef Vec3x<float> Vec3f;
typedef Vec3x<int>   Vec3i;

Vec3f Cross(
    const Vec3f &a,
    const Vec3f &b)
{
    Vec3f res;
    res.x = a.y * b.z - a.z * b.y;
    res.y = a.z * b.x - a.x * b.z;
    res.z = a.x * b.y - a.y * b.x;
    return res;
}

Vec3f Normalize(const Vec3f& a)
{
    const float lenSqr = Dot(a, a);
    const float len    = std::sqrt(lenSqr);
    return a / len;
}

class Mat4f
{
public:

    Mat4f(){}
    Mat4f(float a){ for(int i=0; i<16; i++) GetPtr()[i] = a; }

    const float* GetPtr() const { return reinterpret_cast<const float*>(this); }
    float*       GetPtr()       { return reinterpret_cast<float*>(this);       }

    const float& Get(int r, int c) const { return GetPtr()[r + c*4]; }
    float&       Get(int r, int c)       { return GetPtr()[r + c*4]; }

    void SetRow(int r, float a, float b, float c, float d)
    {
        Get(r, 0) = a;
        Get(r, 1) = b;
        Get(r, 2) = c;
        Get(r, 3) = d;
    }

    void SetRow(int r, const Vec3f &a, float b)
    {
        for(int i=0; i<3; i++)
            Get(r, i) = a.Get(i);

        Get(r, 3) = b;
    }

    Vec3f TransformVector(const Vec3f& aVec) const
    {
        Vec3f res(0);
        for(int r=0; r<3; r++)
            for(int c=0; c<3; c++)
                res.Get(r) += aVec.Get(c) * Get(r, c);

        return res;
    }

    Vec3f TransformPoint(const Vec3f& aVec) const
    {
        float w = Get(3,3);

        for(int c=0; c<3; c++)
            w += Get(3, c) * aVec.Get(c);

        const float invW = 1.f / w;

        Vec3f res(0);

        for(int r=0; r<3; r++)
        {
            res.Get(r) = Get(r, 3);

            for(int c=0; c<3; c++)
                res.Get(r) += aVec.Get(c) * Get(r, c);

            res.Get(r) *= invW;
        }
        return res;
    }

    static Mat4f Zero() { Mat4f res(0); return res; }

    static Mat4f Identity()
    {
        Mat4f res(0);
        for(int i=0; i<4; i++) res.Get(i,i) = 1.f;
        return res;
    }

    static Mat4f Scale(const Vec3f& aScale)
    {
        Mat4f res = Mat4f::Identity();
        for(int i=0; i<3; i++) res.Get(i,i) = aScale.Get(i);
        res.Get(3,3) = 1;
        return res;
    }

    static Mat4f Translate(const Vec3f& aScale)
    {
        Mat4f res = Mat4f::Identity();
        for(int i=0; i<3; i++) res.Get(i,3) = aScale.Get(i);
        res.Get(3,3) = 1;
        return res;
    }

    static Mat4f Perspective(
        float aFov,
        float aNear,
        float aFar)
    {
        // Camera points towards -z.  0 < near < far.
        // Matrix maps z range [-near, -far] to [-1, 1], after homogeneous division.
        float f = 1.f / (std::tan(aFov * PI_F / 360.0f));
        float d = 1.f / (aNear - aFar);

        Mat4f r;
        r.m00 = f;    r.m01 = 0.0f; r.m02 = 0.0f;               r.m03 = 0.0f;
        r.m10 = 0.0f; r.m11 = -f;   r.m12 = 0.0f;               r.m13 = 0.0f;
        r.m20 = 0.0f; r.m21 = 0.0f; r.m22 = (aNear + aFar) * d; r.m23 = 2.0f * aNear * aFar * d;
        r.m30 = 0.0f; r.m31 = 0.0f; r.m32 = -1.0f;              r.m33 = 0.0f;

        return r;
    }
public:

    // m_row_col; stored column major
    float m00, m10, m20, m30;
    float m01, m11, m21, m31;
    float m02, m12, m22, m32;
    float m03, m13, m23, m33;
};

Mat4f operator*(const Mat4f& left, const Mat4f& right)
{
    Mat4f res(0);
    for(int row=0; row<4; row++)
        for(int col=0; col<4; col++)
            for(int i=0; i<4; i++)
                res.Get(row, col) += left.Get(row, i) * right.Get(i, col);

    return res;
}

// Code for inversion taken from:
// http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
Mat4f Invert(const Mat4f& aMatrix)
{
    const float *m = aMatrix.GetPtr();
    float inv[16], det;
    int i;

    inv[0] = m[5] * m[10] * m[15] -
        m[5]  * m[11] * m[14] -
        m[9]  * m[6]  * m[15] +
        m[9]  * m[7]  * m[14] +
        m[13] * m[6]  * m[11] -
        m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
        m[4]  * m[11] * m[14] +
        m[8]  * m[6]  * m[15] -
        m[8]  * m[7]  * m[14] -
        m[12] * m[6]  * m[11] +
        m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
        m[4]  * m[11] * m[13] -
        m[8]  * m[5] * m[15] +
        m[8]  * m[7] * m[13] +
        m[12] * m[5] * m[11] -
        m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
        m[4]  * m[10] * m[13] +
        m[8]  * m[5] * m[14] -
        m[8]  * m[6] * m[13] -
        m[12] * m[5] * m[10] +
        m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
        m[1]  * m[11] * m[14] +
        m[9]  * m[2] * m[15] -
        m[9]  * m[3] * m[14] -
        m[13] * m[2] * m[11] +
        m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
        m[0]  * m[11] * m[14] -
        m[8]  * m[2] * m[15] +
        m[8]  * m[3] * m[14] +
        m[12] * m[2] * m[11] -
        m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
        m[0]  * m[11] * m[13] +
        m[8]  * m[1] * m[15] -
        m[8]  * m[3] * m[13] -
        m[12] * m[1] * m[11] +
        m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
        m[0]  * m[10] * m[13] -
        m[8]  * m[1] * m[14] +
        m[8]  * m[2] * m[13] +
        m[12] * m[1] * m[10] -
        m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
        m[1]  * m[7] * m[14] -
        m[5]  * m[2] * m[15] +
        m[5]  * m[3] * m[14] +
        m[13] * m[2] * m[7] -
        m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
        m[0]  * m[7] * m[14] +
        m[4]  * m[2] * m[15] -
        m[4]  * m[3] * m[14] -
        m[12] * m[2] * m[7] +
        m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
        m[0]  * m[7] * m[13] -
        m[4]  * m[1] * m[15] +
        m[4]  * m[3] * m[13] +
        m[12] * m[1] * m[7] -
        m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
        m[0]  * m[6] * m[13] +
        m[4]  * m[1] * m[14] -
        m[4]  * m[2] * m[13] -
        m[12] * m[1] * m[6] +
        m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
        m[1] * m[7] * m[10] +
        m[5] * m[2] * m[11] -
        m[5] * m[3] * m[10] -
        m[9] * m[2] * m[7] +
        m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
        m[0] * m[7] * m[10] -
        m[4] * m[2] * m[11] +
        m[4] * m[3] * m[10] +
        m[8] * m[2] * m[7] -
        m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
        m[0] * m[7] * m[9] +
        m[4] * m[1] * m[11] -
        m[4] * m[3] * m[9] -
        m[8] * m[1] * m[7] +
        m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
        m[0] * m[6] * m[9] -
        m[4] * m[1] * m[10] +
        m[4] * m[2] * m[9] +
        m[8] * m[1] * m[6] -
        m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return Mat4f::Identity();

    det = 1.f / det;

    Mat4f res;
    for (i = 0; i < 16; i++)
        res.GetPtr()[i] = inv[i] * det;

    return res;
}

#endif //__MATH_HXX__