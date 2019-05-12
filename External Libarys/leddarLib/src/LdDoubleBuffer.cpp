/// *****************************************************************************
/// Module..: Leddar
///
/// \file    LdDoubleBuffer.cpp
///
/// \brief   Implementation of LdDoubleBuffer class
///
/// \author  David Levy
///
/// \since   January 2018
///
/// Copyright (c) 2018 LeddarTech Inc. All rights reserved.
/// *****************************************************************************

#include "LdDoubleBuffer.h"

#include "LtTimeUtils.h"
#include <assert.h>

using namespace LeddarConnection;

/// *****************************************************************************
/// Function: LdDoubleBuffer::LdDoubleBuffer
///
/// \brief   Constructor
///
/// \author  David Levy
///
/// \since   January 2018
/// *****************************************************************************
LdDoubleBuffer::LdDoubleBuffer() : mGetBuffer( new DataBuffer ), mSetBuffer( new DataBuffer ), mTimestamp( nullptr )
{
}

/// *****************************************************************************
/// Function: LdDoubleBuffer::LdDoubleBuffer
///
/// \brief   Copy constructor
///
/// \author  David Levy
///
/// \since   January 2018
/// *****************************************************************************
LdDoubleBuffer::LdDoubleBuffer( const LdDoubleBuffer &aBuffer ) : mGetBuffer( new DataBuffer ), mSetBuffer( new DataBuffer ), mTimestamp( nullptr )
{
    *mGetBuffer = *aBuffer.mGetBuffer;
    *mSetBuffer = *aBuffer.mSetBuffer;
    mTimestamp = aBuffer.mTimestamp;
}

/// *****************************************************************************
/// Function: LdDoubleBuffer::LdDoubleBuffer
///
/// \brief   = Operator
///
/// \author  David Levy
///
/// \since   January 2018
/// *****************************************************************************
LdDoubleBuffer &LeddarConnection::LdDoubleBuffer::operator=( const LdDoubleBuffer &aBuffer )
{
    if( this == &aBuffer )
        return *this;

    *mGetBuffer = *aBuffer.mGetBuffer;
    *mSetBuffer = *aBuffer.mSetBuffer;
    mTimestamp = aBuffer.mTimestamp;
    return *this;
}

/// *****************************************************************************
/// Function: LdDoubleBuffer::~LdDoubleBuffer
///
/// \brief   Destructor - Delete the pointer allocated in the constructor
///
/// \author  David Levy
///
/// \since   January 2018
/// *****************************************************************************
LdDoubleBuffer::~LdDoubleBuffer()
{
    delete mGetBuffer;
    delete mSetBuffer;
    mGetBuffer = nullptr;
    mSetBuffer = nullptr;
}

/// *****************************************************************************
/// Function: LdDoubleBuffer::Init
///
/// \brief   Initialize the buffer. Must be called before using the class.
///
/// \param aGetBuffer : Pointer to the first buffer
/// \param aSetBuffer : Pointer to the other buffer
/// \param aTimestamp : (optionnal) pointer to the timestamp integer property that will be associated to the buffers
///
/// \author  David Levy
///
/// \since   January 2018
/// *****************************************************************************
void
LdDoubleBuffer::Init( void *aGetBuffer, void *aSetBuffer, LeddarCore::LdIntegerProperty *aTimestamp )
{
    mGetBuffer->mBuffer = aGetBuffer;
    mSetBuffer->mBuffer = aSetBuffer;
    mTimestamp          = aTimestamp;

    //Timestamp index 0 is Get buffer, index 1 is Set buffer
    if( mTimestamp )
    {
        mTimestamp->SetCount( 2 );
        mTimestamp->SetValue( 1, 0 );
    }
}

/// *****************************************************************************
/// Function: LdDoubleBuffer::Swap
///
/// \brief   Swap the two data buffer and their corresponding timestamp
///
/// \author  David Levy
///
/// \since   January 2018
/// *****************************************************************************
void LdDoubleBuffer::Swap()
{
    if( !mGetBuffer->mBuffer || !mSetBuffer->mBuffer )
        assert( false ); //Not initialized !

    mSetBuffer->mBusy = true;

    while( mGetBuffer->mBusy )
    {
        LeddarUtils::LtTimeUtils::WaitBlockingMicro( 1 );
    }

    mGetBuffer->mBusy = true;

    DataBuffer *lTempBuffer = mSetBuffer;
    mSetBuffer = mGetBuffer;
    mGetBuffer = lTempBuffer;

    mGetBuffer->mBusy = false;
    mSetBuffer->mBusy = false;

    if( mTimestamp && mTimestamp->Count() == 2 )
    {
        int64_t lOldTimeStamp0 = mTimestamp->Value( 0 );
        mTimestamp->SetValue( 0, mTimestamp->Value( 1 ) );
        mTimestamp->SetValue( 1, lOldTimeStamp0 );
    }
}

/// *****************************************************************************
/// Function: LdDoubleBuffer::GetTimeStamp
///
/// \brief   Get the timestamp of the buffer
///
/// \param aBuffer : Get or Set buffer
///
/// \author  David Levy
///
/// \since   January 2018
/// *****************************************************************************
uint32_t
LdDoubleBuffer::GetTimestamp( eBuffer aBuffer )
{
    if( mTimestamp && ( mTimestamp->GetCount() < 2 || aBuffer == B_GET ) )
    {
        return static_cast<uint32_t>( mTimestamp->Value( 0 ) );
    }
    else
    {
        return static_cast<uint32_t>( mTimestamp->Value( 1 ) );
    }
}

/// *****************************************************************************
/// Function: LdDoubleBuffer::SetTimestamp
///
/// \brief   Set the timestamp of the buffer
///
/// \param aBuffer : Get or Set buffer
///
/// \author  David Levy
///
/// \since   January 2018
/// *****************************************************************************
void LdDoubleBuffer::SetTimestamp( uint32_t aTimestamp )
{
    if( mTimestamp && mTimestamp->GetCount() < 2 )
    {
        mTimestamp->SetValue( 0, aTimestamp );
    }
    else
    {
        mTimestamp->SetValue( 1, aTimestamp );
    }
}