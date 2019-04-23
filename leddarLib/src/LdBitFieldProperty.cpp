// *****************************************************************************
// Module..: Leddar
//
/// \file    LdBitFieldProperty.cpp
///
/// \brief   Definition of LdBitFieldProperty class.
///          This property make easy to manipulates individual bits.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdBitFieldProperty.h"

#include "LtStringUtils.h"

#include <sstream>
#include <string>

// *****************************************************************************
// Function: LdBitFieldProperty::LdBitFieldProperty
//
/// \brief   Constructor.
///
/// \param   aCategory    See LdProperty.
/// \param   aFeatures    See LdProperty.
/// \param   aId          See LdProperty.
/// \param   aDeviceId    See LdProperty.
/// \param   aUnitSize    See LdProperty.
/// \param   aDescription See LdProperty.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

LeddarCore::LdBitFieldProperty::LdBitFieldProperty( LdProperty::eCategories aCategory, uint32_t aFeatures, uint32_t aId,
        uint16_t aDeviceId, uint32_t aUnitSize, const std::string &aDescription ) :
    LdProperty( LdProperty::TYPE_BITFIELD, aCategory, aFeatures, aId, aDeviceId, aUnitSize, sizeof( uint32_t ), aDescription ),
    mDoNotEmitSignal( false ),
    mExclusivityMask( 0 )
{
}


// *****************************************************************************
// Function: LdBitFieldProperty::MaskToBit
//
/// \brief   Static function to convert a single bit mask to its bit index.
///
/// \param   aMask  The mask value. Must have only one bit at 1 or an exception
///                 will be thrown.
///
/// \return  The bit index from 0 to 31 corresponding to aMask.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

uint8_t
LeddarCore::LdBitFieldProperty::MaskToBit( uint32_t aMask )
{
    if( std::bitset<32>( aMask ).count() > 1 )
    {
        throw std::logic_error( "More than one bit are set." );
    }

    for( uint8_t i = 0; i < 32; ++i )
    {
        if( aMask == ( 1u << i ) )
        {
            return i;
        }
    }

    return 0;
}

// *****************************************************************************
// Function: LdBitFieldProperty::SetValue
//
/// \brief   Change the value at the given index.
///
/// This is a raw acces to set the whole value at once. SetBit and ResetBit
/// can be used to change a single bit.
///
/// \param   aIndex  Index in array of value to change.
/// \param   aValue  New value to write.
///
/// \exception std::out_of_range Index not valid, verify property count.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdBitFieldProperty::SetValue( size_t aIndex, uint32_t aValue )
{
    if( !ValidateExclusivity( std::bitset<32>( aValue ) ) )
    {
        throw std::logic_error( "Several exclusive bits are set." );
    }

    // Initialize the count to 1 on the fist SetValue if not done before.
    if( GetCount() == 0 && aIndex == 0 )
    {
        SetCount( 1 );
    }

    uint32_t *lValues = reinterpret_cast<uint32_t *>( Storage() );

    if( aIndex >= GetCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( GetId(), 16 ) );
    }

    if( lValues[aIndex] != aValue )
    {
        lValues[aIndex] = aValue;

        if( !mDoNotEmitSignal )
        {
            EmitSignal( LdObject::VALUE_CHANGED );
        }
    }
}


// *****************************************************************************
// Function: LdBitFieldProperty::GetStringValue
//
/// \brief   Display the value in string format
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

std::string
LeddarCore::LdBitFieldProperty::GetStringValue( size_t aIndex ) const
{
    std::ostringstream lStrStream;
    lStrStream << Value( aIndex );

    return std::string( lStrStream.str() );
}

// *****************************************************************************
// Function: LdBitFieldProperty::SetStringValue
//
/// \brief   Property writer for the value as text.
///
///
/// \param   aIndex  Index of value to write.
/// \param   aValue  The new value.
///
/// \exception std::invalid_argument No conversion could be be performed ( from std::stof )
/// \exception std::out_of_range Value out of range ( from std::stof )
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
void
LeddarCore::LdBitFieldProperty::SetStringValue( size_t aIndex, const std::string &aValue )
{
    uint32_t lValue = Value( aIndex );

    if( aValue.length() > sizeof( uint32_t ) * 8 )
    {
        throw std::out_of_range( "String too long." );
    }

    // SetBit must not emit a ValueChanged signal
    mDoNotEmitSignal = true;

    uint8_t bitIndex = static_cast<uint8_t>( aValue.length() - 1 );

    for( size_t i = 0; i < aValue.length(); ++i )
    {
        char lChar = aValue[ i ];

        if( lChar != '0' && lChar != '1' && lChar != 'x' )
        {
            mDoNotEmitSignal = false;
            throw( std::invalid_argument( "Invalid argument. The string can only contains 0, 1 and x characters." ) );
        }

        if( lChar == '1' )
        {
            SetBit( aIndex, bitIndex );
        }

        if( lChar == '0' )
        {
            ResetBit( aIndex, bitIndex );
        }

        --bitIndex;
    }

    mDoNotEmitSignal = false;

    uint32_t lNewValue = Value( aIndex );

    if( lNewValue != lValue )
    {
        EmitSignal( LdObject::VALUE_CHANGED );
    }
}

// *****************************************************************************
// Function: LdBitFieldProperty::Value
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
uint32_t
LeddarCore::LdBitFieldProperty::Value( size_t aIndex ) const
{
    if( aIndex >= GetCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( GetId() ) );
    }

    return reinterpret_cast<const uint32_t *>( CStorage() )[ aIndex ];
}

/// *****************************************************************************
/// Function: LdBitFieldProperty::ValidateExclusivity
///
/// \brief   True if only one bit is set in the exclusivity mask
///
/// \param   aValue  Value to test.
///
/// \author  David Levy
///
/// \since   June 2018
/// *****************************************************************************
bool
LeddarCore::LdBitFieldProperty::ValidateExclusivity( std::bitset<32> aValue )
{
    if( ( aValue &= mExclusivityMask ).count() > 1 )
    {
        return false;
    }
    else
    {
        return true;
    }
}