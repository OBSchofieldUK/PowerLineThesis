// *****************************************************************************
// Module..: Leddar
//
/// \file    LdResultEchoes.cpp
///
/// \brief   Echoes result.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdResultEchoes.h"
#include "LtTimeUtils.h"

#include <sstream>
using namespace LeddarConnection;

// *****************************************************************************
// Function: LdResultEchoes::LdResultEchoes
//
/// \brief   Constructor.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************
LdResultEchoes::LdResultEchoes( void ) :
    mIsInitialized( false ),
    mDistanceScale( 0 ),
    mAmplitudeScale( 0 ),
    mCurrentLedPower( 0 )
{

}

// *****************************************************************************
// Function: LdResultEchoes::~LdResultEchoes
//
/// \brief   Destructor.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************
LdResultEchoes::~LdResultEchoes()
{
}

// *****************************************************************************
// Function: LdResultEchoes::Init
//
/// \brief   Initialise the result object. This function need to be called before use.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************
void
LdResultEchoes::Init( uint32_t aDistanceScale, uint32_t aAmplitudeScale, uint32_t aMaxDetections )
{
    if( !mIsInitialized )
    {
        // Max detection need to be over 0
        if( aMaxDetections == 0 )
        {
            assert( 0 );
        }

        mEchoBuffer1.mEchoes.resize( aMaxDetections );
        mEchoBuffer2.mEchoes.resize( aMaxDetections );

        LdDoubleBuffer::Init( &mEchoBuffer1, &mEchoBuffer2, LdResultProvider::mTimestamp );

        mDistanceScale = aDistanceScale;
        mAmplitudeScale = aAmplitudeScale;

        mIsInitialized = true;
    }
}

// *****************************************************************************
// Function: LdResultEchoes::GetEchoCount
//
/// \brief   Get the number of echoes of the buffer
///
/// \author  David Levy
///
/// \since   January 2018
// *****************************************************************************
uint32_t
LdResultEchoes::GetEchoCount( eBuffer aBuffer ) const
{
    if( !mIsInitialized )
    {
        assert( 0 );
    }

    return static_cast< EchoBuffer * >( aBuffer == B_GET ? mGetBuffer->mBuffer : mSetBuffer->mBuffer )->mCount;
}

// *****************************************************************************
// Function: LdResultEchoes::GetEchoes
//
/// \brief   Get echoes vector
///
/// \author  David Levy
///
/// \since   January 2018
// *****************************************************************************
std::vector<LdEcho> *
LdResultEchoes::GetEchoes( eBuffer aBuffer )
{
    if( !mIsInitialized )
    {
        assert( 0 );
    }

    return &static_cast< EchoBuffer * >( aBuffer == B_GET ? mGetBuffer->mBuffer : mSetBuffer->mBuffer )->mEchoes;
}

// *****************************************************************************
// Function: LdResultEchoes::GetEchoDistance
//
/// \brief   Get echoes distance at index
///
/// \author  David Levy
///
/// \since   January 2018
// *****************************************************************************
float
LdResultEchoes::GetEchoDistance( size_t aIndex ) const
{
    if( !mIsInitialized )
    {
        assert( 0 );
    }

    return static_cast<float>( static_cast< EchoBuffer * >( mGetBuffer->mBuffer )->mEchoes[aIndex].mDistance ) / mDistanceScale;
}

// *****************************************************************************
// Function: LdResultEchoes::GetEchoAmplitude
//
/// \brief   Get echoes amplitude at index
///
/// \author  David Levy
///
/// \since   January 2018
// *****************************************************************************
float
LdResultEchoes::GetEchoAmplitude( size_t aIndex ) const
{
    if( !mIsInitialized )
    {
        assert( 0 );
    }

    return static_cast<float>( static_cast< EchoBuffer * >( mGetBuffer->mBuffer )->mEchoes[aIndex].mAmplitude ) / mAmplitudeScale;
}

// *****************************************************************************
// Function: LdResultEchoes::GetEchoBase
//
/// \brief   Get echoes base at index
///
/// \author  David Levy
///
/// \since   January 2018
// *****************************************************************************
float
LdResultEchoes::GetEchoBase( size_t aIndex ) const
{
    if( !mIsInitialized )
    {
        assert( 0 );
    }

    return static_cast<float>( static_cast< EchoBuffer * >( mGetBuffer->mBuffer )->mEchoes[aIndex].mBase ) / mAmplitudeScale;
}

#ifdef _DEBUG
// *****************************************************************************
// Function: LdResultEchoes::ToString
//
/// \brief   Format the result into string (for deguging).
///
/// \return  Formatted result.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************
std::string
LdResultEchoes::ToString( void )
{
    std::stringstream lResult;
    Lock( B_GET );

    std::vector<LdEcho> lEchoes = static_cast< EchoBuffer * >( mGetBuffer->mBuffer )->mEchoes;

    for( uint32_t i = 0; i < static_cast< EchoBuffer * >( mGetBuffer->mBuffer )->mCount; ++i )
    {
        lResult << "[" << lEchoes[ i ].mChannelIndex << "]:\t " << lEchoes[ i ].mAmplitude << "\t " << lEchoes[ i ].mDistance << std::endl;
    }

    UnLock( B_GET );
    return lResult.str();
}

#endif

