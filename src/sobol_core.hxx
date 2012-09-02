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
 *
 * I would like to thank Leonhard Gruenschloss (leonhard@gruenschloss.org),
 * for many good comments and suggestions, as well as some pieces of code,
 * used in sobol.hxx (which does contain his license).
 *
 */


#ifndef __SOBOL_CORE_HXX__
#define __SOBOL_CORE_HXX__

#include <vector>
#include "new-joe-kuo-6.21201.h"
#include <cassert>

namespace sobol
{
class Matrices
{
public:
    Matrices(uint aNumberOfBits, uint aNumberOfDimensions)
        : mNumberOfBits(aNumberOfBits),
        mNumberOfDimensions(aNumberOfDimensions)
    {
        if(mNumberOfDimensions == 0) return;

        assert(mNumberOfBits >= 32);
        mMatrices64.resize(mNumberOfBits * mNumberOfDimensions);
        mMatrices32.resize(mNumberOfBits * mNumberOfDimensions);
    }

    void GenerateMatrices()
    {
        if(mNumberOfDimensions == 0) return;

        // First dimension
        for(uint i=0; i<mNumberOfBits; i++)
        {
            mMatrices64[i] = uint64(1) << uint64(mNumberOfBits - 1 - i);
        }

        std::vector<uint64> m;
        for(uint dim = 1; dim < mNumberOfDimensions; dim++)
        {
            // load initial direction numbers
            int *dirNumbers = SobolDirectionNumers[dim - 1];

            uint s, a;
            s = (uint)dirNumbers[0];
            a = (uint)dirNumbers[1];

            // the polynomial does not contain the x^s element,
            // so just add it to avoid special handling
            m.resize(s);
            uint64 *V = &mMatrices64[dim * mNumberOfBits];

            if(mNumberOfBits <= s)
            {
                for(uint i=0; i<mNumberOfBits; i++)
                    V[i] =
                    uint64(dirNumbers[i+2]) << uint64(mNumberOfBits - 1 - i);
            }
            else
            {
                for(uint i=0; i<s; i++)
                    V[i] =
                    uint64(dirNumbers[i+2]) << uint64(mNumberOfBits - 1 - i);

                for(uint i=s; i<mNumberOfBits; i++)
                {
                    V[i] = V[i-s] ^ (V[i-s] >> uint64(s));
                    for (uint k=0;k<s-1;k++)
                        V[i] ^= (((a >> (s-2-k)) & 1) * V[i-k-1]);
                }
            }
        }

        // Convert from 64b to 32b matrix
        uint64 shiftBy = mNumberOfBits - 32;
        for(size_t i=0; i < mMatrices64.size(); i++)
        {
            mMatrices32[i] = uint(mMatrices64[i] >> shiftBy);
        }
    }

    uint Sample32(
        uint64       aIndex,
        const uint   aDimension,
        const uint   aScramble = uint(0)) const
    {
        assert(aDimension < mNumberOfDimensions);

        uint result = aScramble;
        for(uint i = mNumberOfBits * aDimension;
            aIndex; aIndex >>= 1, i++)
        {
            if(aIndex & 1)
                result ^= mMatrices32[i];
        }
        return result;
    }

    uint64 Sample64(
        uint64       aIndex,
        const uint   aDimension,
        const uint64 aScramble = uint(0)) const
    {
        assert(aDimension < mNumberOfDimensions);

        uint64 result = aScramble >> (64 - mNumberOfBits);
        for(uint i = mNumberOfBits * aDimension;
            aIndex; aIndex >>= 1, i++)
        {
            if(aIndex & 1)
                result ^= mMatrices64[i];
        }
        return result;
    }

public:
    const uint mNumberOfBits;
    const uint mNumberOfDimensions;
private:
    std::vector<uint64> mMatrices64;
    std::vector<uint>   mMatrices32;
};


}

#endif //__SOBOL_CORE_HXX__
