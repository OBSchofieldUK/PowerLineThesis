// *****************************************************************************
// Module..: Leddar
//
/// \file    LdIntegerProperty.cpp
///
/// \brief   Definition of functions for class LdIntegerProperty.
///
/// \author  Patrick Boulay
///
/// \since   February 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdIntegerProperty.h"
#include "LtStringUtils.h"

#include <cassert>
#include <limits>


// *****************************************************************************
// Function: LdIntegerProperty::LdIntegerProperty
//
/// \brief   Constructor.
///
/// The limits are set to the maximum range for the unit size.
///
/// \param   aFeatures    See LdProperty.
/// \param   aId          See LdProperty.
/// \param   aDeviceId    See LdProperty.
/// \param   aUnitSize    See LdProperty.
/// \param   aCategory    See LdProperty.
/// \param   aDescription See LdProperty.
/// \param   aSigned      Is the integer signed?
///
/// \author  Louis Perreault
///
/// \since   August 2014
// *****************************************************************************

LeddarCore::LdIntegerProperty::LdIntegerProperty( LdProperty::eCategories aCategory, uint32_t aFeatures, uint32_t aId,
        uint16_t aDeviceId, uint32_t aUnitSize, const std::string &aDescription, const bool aSigned ) :
    LdProperty( LdProperty::TYPE_INTEGER, aCategory, aFeatures, aId, aDeviceId, aUnitSize, sizeof( int32_t ), aDescription ),
    mMinValue( 0 ),
    mMaxValue( 0 ),
    mSigned( aSigned )
{
    assert( ( aUnitSize == 1 ) || ( aUnitSize == 2 ) || ( aUnitSize == 4 ) );

    switch( UnitSize() )
    {
        case 1:
            if( mSigned )
            {
                mMaxValue = std::numeric_limits<int8_t>::max();
                mMinValue = std::numeric_limits<int8_t>::min();
            }
            else
                mMaxValue = std::numeric_limits<uint8_t>::max();

            break;

        case 2:
            if( mSigned )
            {
                mMaxValue = std::numeric_limits<int16_t>::max();
                mMinValue = std::numeric_limits<int16_t>::min();
            }
            else
                mMaxValue = std::numeric_limits<uint16_t>::max();

            break;

        case 4:
            if( mSigned )
            {
                mMaxValue = std::numeric_limits<int32_t>::max();
                mMinValue = std::numeric_limits<int32_t>::min();
            }
            else
                mMaxValue = std::numeric_limits<uint32_t>::max();

            break;
    }

}

// *****************************************************************************
// Function: LdIntegerProperty::SetLimits
//
/// \brief   Change the minimum and maximum allowed values.
///
/// The current value(s) will be clipped to the new limits.
///
/// \param   aMin  The minimum allowed value.
/// \param   aMax  The maximum allowed value.
///
/// \author  Louis Perreault
///
/// \since   August 2014
// *****************************************************************************
void
LeddarCore::LdIntegerProperty::SetLimits( int64_t aMin, int64_t aMax )
{
    if( aMin > aMax )
    {
        throw std::invalid_argument( "Invalid min value is higher than the max value." );
    }

    if( ( aMin != mMinValue ) || ( aMax != mMaxValue ) )
    {
        mMinValue = aMin;
        mMaxValue = aMax;

        // Clip current values
        const size_t lCount = Count();

        if( lCount > 0 )
        {
            const int32_t *lValues = reinterpret_cast<const int32_t *>( Storage() );

            for( size_t i = 0; i < lCount; ++i )
            {
                if( lValues[ i ] < mMinValue )
                {
                    SetValue( i, mMinValue );
                }
                else if( lValues[ i ] > mMaxValue )
                {
                    SetValue( i, mMaxValue );
                }
            }
        }

        EmitSignal( LdObject::LIMITS_CHANGED );
    }
}

// *****************************************************************************
// Function: LdIntegerProperty::SetValue
//
/// \brief   Change the current value at the given index.
///
/// An exception will be throwned if the new value is not within the limits.
///
/// \param   aIndex  Index of value to change.
/// \param   aValue  The new value.
///
/// \exception std::out_of_range Index not valid, verify property count.
///
/// \author  Louis Perreault
///
/// \since   August 2014
// *****************************************************************************

void
LeddarCore::LdIntegerProperty::SetValue( size_t aIndex, int64_t aValue )
{
    // Initialize the count to 1 on the fist SetValue if not done before.
    if( GetCount() == 0 && aIndex == 0 )
    {
        SetCount( 1 );
    }

    if( aIndex >= GetCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( GetId() ) );
    }

    if( mSigned )
    {
        int32_t *lValues = reinterpret_cast<int32_t *>( Storage() );

        if( lValues[aIndex] != aValue )
        {
            if( ( aValue < mMinValue )
                    || ( aValue > mMaxValue ) )
            {
                throw std::out_of_range( "Value out of range. Check min and max value." );
            }

            lValues[aIndex] = static_cast<int32_t>( aValue );
            EmitSignal( LdObject::VALUE_CHANGED );
        }
    }
    else
    {
        uint32_t *lValues = reinterpret_cast<uint32_t *>( Storage() );

        if( lValues[aIndex] != aValue )
        {
            if( ( aValue < mMinValue )
                    || ( aValue > mMaxValue ) )
            {
                throw std::out_of_range( "Value out of range. Check min and max value." );
            }

            lValues[aIndex] = static_cast<uint32_t>( aValue );
            EmitSignal( LdObject::VALUE_CHANGED );
        }
    }
}

// *****************************************************************************
// Function: LdIntegerProperty::GetStringValue
//
/// \brief   Display the value in string format
///
/// \author  Patrick Boulay
///
/// \since   February 2016
// *****************************************************************************

std::string
LeddarCore::LdIntegerProperty::GetStringValue( size_t aIndex ) const
{
    return LeddarUtils::LtStringUtils::IntToString( Value( aIndex ) );
}


// *****************************************************************************
// Function: LdIntegerProperty::SetStringValue
//
/// \brief   Property writer for the value as text.
///
/// See SetStringValue with 3 arguments for more information.
/// This function is to be compliant with the virtual SetStringValue function.
///
/// \param   aIndex  Index of value to write.
/// \param   aValue  The new value.
///
/// \exception std::invalid_argument No conversion could be be performed ( from std::stoi )
/// \exception std::out_of_range Value out of range ( from std::stoi )
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdIntegerProperty::SetStringValue( size_t aIndex, const std::string &aValue )
{
    SetStringValue( aIndex, aValue, 10 );
}

// *****************************************************************************
// Function: LdIntegerProperty::SetStringValue
//
/// \brief   Property writer for the value as text.
///
/// The text must be valid as a integer value in base 10. The value will only
/// be changed if the new text, once standardized is different than the
/// current text value (so changes must be seen within the number of
/// decimals configured).
///
/// \param   aIndex  Index of value to write.
/// \param   aValue  The new value.
/// \param   aBase   Base of the number.
///
/// \exception std::invalid_argument Invalid input string, no conversion could be performed.
/// \exception std::overflow_error The value exceed maximum of Int value.
/// \exception std::underflow_error The value is under the minimum of Int value.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
void
LeddarCore::LdIntegerProperty::SetStringValue( size_t aIndex, const std::string &aValue, uint8_t aBase )
{
    const std::string lCurrent = GetStringValue( aIndex );

    if( lCurrent != aValue )
    {
        SetValue( aIndex, static_cast<int32_t>( LeddarUtils::LtStringUtils::StringToInt( aValue, aBase ) ) );
    }
}

// *****************************************************************************
// Function: LdIntegerProperty::Value
//
/// \brief   Return the property value
///
/// \param   aIndex  Index of value.
///
/// \exception std::out_of_range Value out of range ( from std::stoi )
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
int64_t
LeddarCore::LdIntegerProperty::Value( size_t aIndex ) const
{
    if( aIndex >= GetCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( GetId(), 16 ) );
    }

    if( mSigned )
        return reinterpret_cast<const int32_t *>( CStorage() )[ aIndex ];
    else
        return reinterpret_cast<const uint32_t *>( CStorage() )[ aIndex ];
}

/// *****************************************************************************
/// Function: LdIntegerProperty::ValueT
///
/// \brief   Return the property value as the requested type
///
/// \param   aIndex  Index of value.
///
/// \exception std::out_of_range Value out of range ( wrong index )
/// \exception std::out_of_range Value out of range ( if return type is not large enough )
///
/// \author  David Levy
///
/// \since   March 2018
/// *****************************************************************************
template<class T>
T LeddarCore::LdIntegerProperty::ValueT( size_t aIndex ) const
{
    int64_t lValue = Value( aIndex );

    if( lValue > std::numeric_limits<T>::max() || lValue < std::numeric_limits<T>::min() )
        throw std::out_of_range( "Value is too large for return type:" + LeddarUtils::LtStringUtils::IntToString( lValue ) );
    else
        return static_cast< T >( lValue );
}

//Template specialisation so it can be defined in the cpp file
template uint8_t LeddarCore::LdIntegerProperty::ValueT( size_t aIndex ) const;
template int8_t LeddarCore::LdIntegerProperty::ValueT( size_t aIndex ) const;
template uint16_t LeddarCore::LdIntegerProperty::ValueT( size_t aIndex ) const;
template int16_t LeddarCore::LdIntegerProperty::ValueT( size_t aIndex ) const;
template uint32_t LeddarCore::LdIntegerProperty::ValueT( size_t aIndex ) const;
template int32_t LeddarCore::LdIntegerProperty::ValueT( size_t aIndex ) const;