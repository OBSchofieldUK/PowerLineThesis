// *****************************************************************************
// Module..: Leddar
//
/// \file    LdBoolProperty.cpp
///
/// \brief   Definition of LdBoolProperty class.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdBoolProperty.h"

#include "LtStringUtils.h"

#include <string>

// *****************************************************************************
// Function: LdBoolProperty::LdBoolProperty
//
/// \brief   Constructor.
///
/// \param   aCategory    See LdProperty.
/// \param   aFeatures    See LdProperty.
/// \param   aId          See LdProperty.
/// \param   aDeviceId    See LdProperty.
/// \param   aDescription See LdProperty.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

LeddarCore::LdBoolProperty::LdBoolProperty( LdProperty::eCategories aCategory, uint32_t aFeatures, uint32_t aId, uint16_t aDeviceId, const std::string &aDescription ) :
    LdProperty( LdProperty::TYPE_BOOL, aCategory, aFeatures, aId, aDeviceId, 1, sizeof( bool ), aDescription )
{
}


// *****************************************************************************
// Function: LdBoolProperty::SetValue
//
/// \brief   Change the value at the given index.
///
/// \param   aIndex  Index in array of value to change.
/// \param   aValue  New value to write.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdBoolProperty::SetValue( size_t aIndex, bool aValue )
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

    bool *lValues = reinterpret_cast<bool *>( Storage() );

    if( lValues[ aIndex ] != aValue )
    {
        lValues[ aIndex ] = aValue;
        EmitSignal( LdObject::VALUE_CHANGED );
    }
}

// *****************************************************************************
// Function: LdBoolProperty::GetStringValue
//
/// \brief   Display the value in string format
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

std::string
LeddarCore::LdBoolProperty::GetStringValue( size_t aIndex ) const
{
    return ( Value( aIndex ) == true ? "true" : "false" );
}

// *****************************************************************************
// Function: LdBoolProperty::SetStringValue
//
/// \brief   Property writer for the value as text. Possible value: true and false (lower case)
///
/// \param   aIndex  Index of value to write.
/// \param   aValue  The new value.
///
/// \exception std::invalid_argument If the string is not valid.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdBoolProperty::SetStringValue( size_t aIndex, const std::string &aValue )
{
    bool lNewValue = false;

    if( aValue == "true" )
    {
        lNewValue = true;
    }
    else if( aValue == "false" )
    {
        lNewValue = false;
    }
    else
    {
        throw( std::invalid_argument( "Invalid string value." ) );
    }

    SetValue( aIndex, lNewValue );
    return;
}

// *****************************************************************************
// Function: LdBoolProperty::Value
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

bool
LeddarCore::LdBoolProperty::Value( size_t aIndex ) const
{
    if( aIndex >= GetCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( GetId() ) );
    }

    return reinterpret_cast<const bool *>( CStorage() )[ aIndex ];
}