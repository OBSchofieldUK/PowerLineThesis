// *****************************************************************************
// Module..: Leddar
//
/// \file    LdDoubleBuffer.h
///
/// \brief   LdDoubleBuffer class definition
/// \brief      Class to manage dual buffering.
/// \bried      This class shouldnt be instantiated, but inherited. The child class needs to handle the (de)initialisation of mGetBuffer->nBuffer and mSetBuffer->nBuffer
///
/// \author  David Levy
///
/// \since   January 2018
//
// Copyright (c) 2018 LeddarTech Inc. All rights reserved.
// *****************************************************************************
#pragma once

#include "LtDefines.h"
#include "LdIntegerProperty.h"

namespace LeddarConnection
{
    typedef struct DataBuffer
    {
        DataBuffer() : mBuffer( nullptr ), mBusy( false ) {}
        void        *mBuffer; //Actual data buffer. The class doesnt know the type, so the child class does all actual manipulation (other than the swap)
        bool        mBusy;
    } DataBuffer;

    enum eBuffer
    {
        B_SET,
        B_GET
    };

    class LdDoubleBuffer
    {
    public:
        virtual ~LdDoubleBuffer();
        void Init( void *aGetBuffer, void *aSetBuffer, LeddarCore::LdIntegerProperty *aTimestamp = nullptr );

        void Swap();
        void Lock( eBuffer aBuffer ) { aBuffer == B_GET ? mGetBuffer->mBusy = true : mSetBuffer->mBusy = true; }
        void UnLock( eBuffer aBuffer ) { aBuffer == B_GET ? mGetBuffer->mBusy = false : mSetBuffer->mBusy = false; }

        uint32_t    GetTimestamp( eBuffer aBuffer = B_GET );
        void        SetTimestamp( uint32_t aTimestamp );

    protected:
        LdDoubleBuffer();
        LdDoubleBuffer( const LdDoubleBuffer &aBuffer );
        LdDoubleBuffer &operator=( const LdDoubleBuffer &aBuffer );

        DataBuffer *mGetBuffer;
        DataBuffer *mSetBuffer;

    private:
        LeddarCore::LdIntegerProperty *mTimestamp;
    };
}

