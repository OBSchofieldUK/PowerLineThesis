// *****************************************************************************
// Module..: Leddar
//
/// \file    LdResultEchoes.h
///
/// \brief   Echoes result.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "LdIntegerProperty.h"
#include "LdResultProvider.h"
#include "LdDoubleBuffer.h"

#include <cassert>


namespace LeddarConnection
{
    struct LdEcho
    {
        int32_t  mDistance;
        uint32_t mAmplitude;
        uint32_t mBase;
        uint16_t mMaxIndex;
        uint16_t mChannelIndex;
        uint16_t mFlag;
        uint32_t mAmplitudeLowScale;
        uint32_t mSaturationWidth;
    };

    typedef struct EchoBuffer
    {
        EchoBuffer(): mCount( 0 ) {};
        std::vector<LdEcho> mEchoes;
        uint32_t    mCount;
    } EchoBuffer;

    class LdResultEchoes : public LdResultProvider, public LdDoubleBuffer
    {
    public:
        LdResultEchoes( void );
        ~LdResultEchoes();

        using LdDoubleBuffer::GetTimestamp;
        using LdDoubleBuffer::SetTimestamp;

        void Init( uint32_t aDistanceScale, uint32_t aAmplitudeScale, uint32_t aMaxDetections );
        bool IsInitialized( void ) const { return mIsInitialized;  }

        uint32_t GetEchoCount( eBuffer aBuffer = B_GET ) const;
        std::vector<LdEcho> *GetEchoes( eBuffer aBuffer = B_GET );
        float GetEchoDistance( size_t aIndex ) const;
        float GetEchoAmplitude( size_t aIndex ) const;
        float GetEchoBase( size_t aIndex ) const;
        size_t GetEchoesSize( void ) { return static_cast< EchoBuffer * >( mGetBuffer->mBuffer )->mEchoes.size(); }
        void  SetEchoCount( uint32_t aValue ) { static_cast< EchoBuffer * >( mSetBuffer->mBuffer )->mCount = aValue; }
        void  SetCurrentLedPower( uint16_t aValue ) { mCurrentLedPower = aValue; }
        uint16_t GetCurrentLedPower( void ) const { return mCurrentLedPower; }
        uint32_t GetDistanceScale( void ) {return mDistanceScale;}
        uint32_t GetAmplitudeScale( void ) {return mAmplitudeScale;}

#ifdef _DEBUG
        std::string ToString( void );
#endif

    private:
        bool mIsInitialized;
        uint32_t mDistanceScale;
        uint32_t mAmplitudeScale;
        uint16_t mCurrentLedPower;
        EchoBuffer mEchoBuffer1, mEchoBuffer2;
    };
}