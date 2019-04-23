// *****************************************************************************
// Module..: Leddar
//
/// \file    LdEnumProperty.cpp
///
/// \brief   Definition of class LdEnumProperty.
///
/// \author  Patrick Boulay
///
/// \since   February 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdEnumProperty.h"
#include "LtStringUtils.h"
#include <cassert>
#include <limits>

// *****************************************************************************
// Function: LdEnumProperty::LdEnumProperty
//
/// \brief   Constructor.
///
/// \param   aCategory    See LdProperty.
/// \param   aFeatures    See LdProperty.
/// \param   aId          See LdProperty.
/// \param   aDeviceId    See LdProperty.
/// \param   aUnitSize    See LdProperty.
/// \param   aStoreValue  True to store the value of the enum or false to store the index.
/// \param   aDescription See LdProperty.
///
/// \author  Patrick Boulay
///
/// \since   February 2016
// *****************************************************************************

LeddarCore::LdEnumProperty::LdEnumProperty( LdProperty::eCategories aCategory, uint32_t aFeatures, uint32_t aId, uint16_t aDeviceId, uint32_t aUnitSize, bool aStoreValue,
        const std::string &aDescription ) :
    LdProperty( LdProperty::TYPE_ENUM, aCategory, aFeatures, aId, aDeviceId, aUnitSize, sizeof( uint32_t ), aDescription ),
    mStoreValue( aStoreValue )
{
}

// *****************************************************************************
// Function: LdEnumProperty::ValueIndex
//
/// \brief   Index in the enum of the current value.
///
/// \param   aIndex  Index of value to query.
///
/// \return  The index.
///
/// \author  Patrick Boulay
///
/// \since   February 2016
// *****************************************************************************

uint32_t
LeddarCore::LdEnumProperty::ValueIndex( size_t aIndex ) const
{
    const uint32_t  lValue = Value( aIndex );
    const uint32_t lSize = static_cast<uint32_t>( mEnumValues.size() );

    for( uint32_t i = 0; i < lSize; ++i )
    {
        if( mEnumValues[ i ].first == lValue )
        {
            return i;
        }
    }

    throw std::out_of_range( "No index associated to this value." );
}


// *****************************************************************************
// Function: LdEnumProperty::SetValue
//
/// \brief   Set the value at the current array index.
///
/// \param   aIndex  Index of value to write in the array.
/// \param   aValue  The new value to write.
///
/// \author  Patrick Boulay
///
/// \since   February 2016
// *****************************************************************************

void
LeddarCore::LdEnumProperty::SetValue( size_t aIndex, uint32_t aValue )
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


    if( aValue != Value( aIndex ) )
    {
        uint32_t lIndex = static_cast<uint32_t>( mEnumValues.size() - 1 );

        for( std::vector<Pair>::reverse_iterator lIter = mEnumValues.rbegin(); lIter < mEnumValues.rend(); ++lIter )
        {
            if( lIter->first == aValue )
            {
                if( mStoreValue )
                {
                    reinterpret_cast<uint32_t *>( Storage() )[aIndex] = aValue;
                }
                else
                {
                    reinterpret_cast<uint32_t *>( Storage() )[aIndex] = lIndex;
                }

                EmitSignal( LdObject::VALUE_CHANGED );
                return;
            }

            lIndex--;
        }

        // If we get here aValue is not valid (i.e. it is not in the enum).
        throw std::out_of_range( "No associated value found for this enum. Property id: " + LeddarUtils::LtStringUtils::IntToString( GetId() ) );
    }
}

// *****************************************************************************
// Function: LdEnumProperty::GetKeyFromValue
//
/// \brief   Return the key value of the string value
///
/// \param   aValue  String value of the enum
///
/// \return  The associated key for the value
///
/// \author  Patrick Boulay
///
/// \since   February 2016
// *****************************************************************************
uint32_t
LeddarCore::LdEnumProperty::GetKeyFromValue( const std::string &aValue )
{
    for( std::vector<Pair>::iterator lIter = mEnumValues.begin(); lIter < mEnumValues.end(); ++lIter )
    {
        if( lIter->second == aValue )
        {
            return lIter->first;
        }
    }

    throw std::out_of_range( "No associated string value found for this enum." );
}


// *****************************************************************************
// Function: LdEnumProperty::AddEnumPair
//
/// \brief   Add a new entry in the enum.
///
/// \param   aValue  The value that is stored in the property and set in the
///                  device.
/// \param   aText   The text that is displayed to the user for aValue.
///
/// \author  Patrick Boulay
///
/// \since   February 2016
// *****************************************************************************

void
LeddarCore::LdEnumProperty::AddEnumPair( uint32_t aValue, const std::string &aText )
{
    assert( mEnumValues.size() < std::numeric_limits<uint8_t>::max() - 1u );

    mEnumValues.push_back( Pair( aValue, aText ) );
}

// *****************************************************************************
// Function: LdEnumProperty::ClearEnum
//
/// \brief   Clear the enum vector. Use this function carefully.
///
/// \author  Patrick Boulay
///
/// \since   October 2017
// *****************************************************************************

void
LeddarCore::LdEnumProperty::ClearEnum( void )
{
    mEnumValues.clear();
}

// *****************************************************************************
// Function: LdEnumProperty::SetValueIndex
//
/// \brief   Set the value via its index in the enum.
///
/// \param   aArrayIndex  Index of value to write in the array.
/// \param   aEnumIndex   Index in enum of new value.
///
/// \exception std::out_of_range Enum index not valid
///
/// \author  Patrick Boulay
///
/// \since   February 2016
// *****************************************************************************

void
LeddarCore::LdEnumProperty::SetValueIndex( size_t aArrayIndex, uint32_t aEnumIndex )
{
    if( aEnumIndex >= mEnumValues.size() )
    {
        throw std::out_of_range( "Enum index not valid." );
    }

    SetValue( aArrayIndex, mEnumValues[ aEnumIndex ].first );
}

// *****************************************************************************
// Function: LdEnumProperty::GetStringValue
//
/// \brief   Returns the string of the value.
///
/// \param   aIndex  Index of value to query.
///
/// \return  Value in string format
///
/// \exception std::out_of_range Index not valid, verify property count.
///
/// \author  Patrick Boulay
///
/// \since   February 2016
// *****************************************************************************

std::string
LeddarCore::LdEnumProperty::GetStringValue( size_t aIndex ) const
{
    if( aIndex >= GetCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( GetId() ) );
    }

    return mEnumValues[ ValueIndex( aIndex ) ].second;
}


// *****************************************************************************
// Function: LdEnumProperty::SetStringValue
//
/// \brief   Property writer for the value as text.
///
/// \param   aIndex  Index of value to write.
/// \param   aValue  The new value.
///
/// \exception std::out_of_range If the string is not found in the enum.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdEnumProperty::SetStringValue( size_t aIndex, const std::string &aValue )
{
    uint32_t lKey = GetKeyFromValue( aValue );
    SetValue( aIndex, lKey );
}

// *****************************************************************************
// Function: LdEnumProperty::Value
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
LeddarCore::LdEnumProperty::Value( size_t aIndex ) const
{
    if( aIndex >= GetCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count." );
    }

    uint32_t lStoredValue = reinterpret_cast<const uint32_t *>( CStorage() )[ aIndex ];

    if( mStoreValue )
    {
        return lStoredValue;
    }
    else
    {
        // Index is stored, return the associated value
        return mEnumValues[ lStoredValue ].first;
    }
}