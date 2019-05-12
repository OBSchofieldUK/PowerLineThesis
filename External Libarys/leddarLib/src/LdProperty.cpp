// *****************************************************************************
// Module..: Leddar
//
/// \file    LdProperty.cpp
///
/// \brief   Definition of functions for class LdProperty.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdProperty.h"
#include "LtExceptions.h"
#include "LtStringUtils.h"

#include <cassert>
#include <cstring>

// *****************************************************************************
// Function: LdObject::LdProperty
//
/// \brief   Constructor.
///
/// \param   aCategory    Category of the property.
/// \param   aFeatures    Combination of feature bits from the eFeatures enum.
/// \param   aId          The globally unique value identifying this property.
///                       Also used as the file id. Cannot be 0.
/// \param   aDeviceId    The value used by the device to identify this property.
///                       Can be 0 if the property is not involved in communication
///                       with the device.
/// \param   aUnitSize    The number of bytes for each value (for raw storage
///                       for interaction with files and LeddarTech protocol).
/// \param   aStride      The number of bytes for each value in the local
///                       storage.
/// \param   aDescription Name or description of the property (optional)
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

LeddarCore::LdProperty::LdProperty( ePropertyType aPropertyType, eCategories aCategory, uint32_t aFeatures, uint32_t aId,
                                    uint16_t aDeviceId, uint32_t aUnitSize, size_t aStride, const std::string &aDescription ) :
    mFeatures( aFeatures ),
    mPropertyType( aPropertyType ),
    mId( aId ),
    mCategory( aCategory ),
    mDeviceId( aDeviceId ),
    mStride( aStride ),
    mUnitSize( aUnitSize ),
    mDescription( aDescription )
{
    assert( aId && aUnitSize );
}

// *****************************************************************************
// Function: LdProperty::Modified
//
/// \brief   Indicate if the property is modified.
///
/// Modified means the backup values are not equal to the current values and
/// the property has the EDITABLE bit.
///
/// \return  true if modified, false if clean.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

bool
LeddarCore::LdProperty::Modified( void ) const
{
    return ( mStorage != mBackupStorage );
}

// *****************************************************************************
// Function: LdProperty::SetClean
//
/// \brief   Set the backup values to be the current values (so the property
///          is reported as not modified).
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdProperty::SetClean( void )
{
    mBackupStorage = mStorage;
}

// *****************************************************************************
// Function: LdProperty::SetCount
//
/// \brief   Set the array size of this property.
///
/// \param   aValue  The new count.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdProperty::SetCount( size_t aValue )
{
    mStorage.resize( aValue * mStride );
    mBackupStorage.resize( mStorage.size() );
}

// *****************************************************************************
// Function: LdProperty::SetCount
//
/// \brief   Set the array size of this property.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

size_t
LeddarCore::LdProperty::GetCount( void ) const
{
    return mStorage.size() / mStride;
}

// *****************************************************************************
// Function: LdProperty::Restore
//
/// \brief   Cancel changes by writing the backup values back in the current
///          values.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdProperty::Restore( void )
{
    if( Modified() )
    {
        mStorage = mBackupStorage;
        EmitSignal( LdObject::VALUE_CHANGED );
    }
}

// *****************************************************************************
// Function: LdProperty::SetRawStorage
//
/// \brief   Set storage directly in memory
///
/// \param   aBuffer Buffer to copy
/// \param   aCount  Number of element in the buffer
/// \param   aSize   Size of each element in the buffer
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

void
LeddarCore::LdProperty::SetRawStorage( uint8_t *aBuffer, size_t aCount, uint32_t aSize )
{
    if( GetCount() != aCount )
    {
        SetCount( aCount );
    }

    if( aSize == mStride )
    {
        memcpy( static_cast<uint8_t *>( &mStorage[0] ), aBuffer, aSize * aCount );
    }
    else
    {
        if( aSize > sizeof( uint32_t ) )
        {
            throw LeddarException::LtException( "Unable to SetRawStorage, invalid size: " + LeddarUtils::LtStringUtils::IntToString( aSize )
                                                + " id: " + LeddarUtils::LtStringUtils::IntToString( mId, 16 ) );
        }

        for( uint32_t i = 0; i < aCount; i++ )
        {
            uint32_t lValue( 0 );

            if( aSize == sizeof( uint8_t ) )
            {
                lValue = static_cast<uint8_t *>( aBuffer )[i];
            }
            else if( aSize == sizeof( uint16_t ) )
            {
                lValue = reinterpret_cast<uint16_t *>( aBuffer )[i];
            }
            else if( aSize == sizeof( uint32_t ) )
            {
                lValue = reinterpret_cast<uint32_t *>( aBuffer )[i];
            }

            if( mStride == sizeof( uint8_t ) )
            {
                static_cast<uint8_t *>( &mStorage[0] )[i] = lValue;
            }
            else if( mStride == sizeof( uint16_t ) )
            {
                reinterpret_cast<uint16_t *>( &mStorage[0] )[i] = lValue;
            }
            else if( mStride == sizeof( uint32_t ) )
            {
                reinterpret_cast<uint32_t *>( &mStorage[0] )[i] = lValue;
            }
        }
    }

    EmitSignal( LdObject::VALUE_CHANGED );
}

// *****************************************************************************
// Function: LdProperty::SetRawValue
//
/// \brief   Set storage directly in memory
///
/// \param   aIndex Index of value to change.
/// \param   aValue The new value.
///
/// \author  David Levy
///
/// \since   February 2018
// *****************************************************************************
void LeddarCore::LdProperty::SetRawValue( size_t aIndex, int32_t aValue )
{
    if( aValue != RawValue( aIndex ) )
    {
        reinterpret_cast<int32_t *>( Storage() )[aIndex] = aValue;
        EmitSignal( LdObject::VALUE_CHANGED );
    }
}
