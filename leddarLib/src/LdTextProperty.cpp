// *****************************************************************************
// Module..: Leddar
//
/// \file    LdTextProperty.cpp
///
/// \brief   Definition of functions for class LdTextProperty.
///
/// \author  Patrick Boulay
///
/// \since   February 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdTextProperty.h"
#include "LtExceptions.h"
#include "LtStringUtils.h"
#include <cassert>
#include <cstring>

// *****************************************************************************
// Function: LdTextProperty::LdTextProperty
//
/// \brief   Constructor.
///
/// \param   aCategory    See LdProperty.
/// \param   aFeatures    See LdProperty.
/// \param   aId          See LdProperty.
/// \param   aDeviceId    See LdProperty.
/// \param   aMaxLength   The maximum number of characters (not including null-term).
/// \param   aDescription See LdProperty.
/// \param   aType        Encoding (ASCII or UTF16)
///
/// \author  Louis Perreault
///
/// \since   May 2014
// *****************************************************************************

LeddarCore::LdTextProperty::LdTextProperty( LdProperty::eCategories aCategory, uint32_t aFeatures,
        uint32_t aId, uint16_t aDeviceId, uint32_t aMaxLength, eType aType, const std::string &aDescription ) :
    LdProperty( LdProperty::TYPE_TEXT, aCategory, aFeatures, aId, aDeviceId, aMaxLength, aMaxLength, aDescription ),
    mForceUppercase( false ),
    mType( aType )
{
}

// *****************************************************************************
// Function: LdTextProperty::SetTextValue
//
/// \brief   Set the value from a string
///
/// \param   aIndex  Currently not used.
/// \param   aValue  string providing the new content.
///
/// \exception std::out_of_range Index not valid, verify property count.
/// \exception std::length_error String size exceed maxmim length fixed by the property constructor.
///
/// \author  Louis Perreault
///
/// \since   June 2014
// *****************************************************************************

void
LeddarCore::LdTextProperty::SetValue( size_t aIndex, const std::string &aValue )
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

    if( mType == TYPE_ASCII || mType == TYPE_UTF8 )
    {
        size_t lSize = ( aValue.length() > MaxLength() ? MaxLength() : aValue.length() );

        memset( Storage(), 0, MaxLength() );
        memcpy( Storage(), aValue.c_str(), lSize );
    }
    else
    {
        // Convert UTF8 to UTF16 - note : not really but kept for retro compatibility
        size_t lSize = ( aValue.length() > MaxLength() / 2 ? MaxLength() / 2 : aValue.length() );
        memset( Storage(), 0, MaxLength() );

        for( size_t i = 0; i < lSize; ++i )
        {
            reinterpret_cast<uint16_t *>( Storage() )[i] = static_cast<uint16_t>( aValue[i] );
        }
    }

    EmitSignal( LdObject::VALUE_CHANGED );
}

// *****************************************************************************
// Function: LdTextProperty::SetTextValue
//
/// \brief   Set the value from a string
///
/// \param   aIndex  Currently not used.
/// \param   aValue  wstring providing the new content.
///
/// \exception std::out_of_range Index not valid, verify property count.
/// \exception std::length_error String size exceed maxmim length fixed by the property constructor.
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

void
LeddarCore::LdTextProperty::SetValue( size_t aIndex, const std::wstring &aValue )
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

    if( mType == TYPE_ASCII )
    {
        std::string lASCIIString( aValue.begin(), aValue.end() );
        size_t lSize = ( lASCIIString.length() > MaxLength() ? MaxLength() : lASCIIString.length() );

        memset( Storage(), 0, MaxLength() );
        memcpy( Storage(), lASCIIString.c_str(), lSize );
    }
    else if( mType == TYPE_UTF16 )
    {
        memset( Storage(), 0, MaxLength() );

        for( size_t i = 0; i < aValue.size(); ++i )
        {
            reinterpret_cast< uint16_t * >( Storage() )[i] = static_cast< uint16_t >( aValue[i] );
        }

    }
    else
    {
#ifdef _WIN32
        memset( Storage(), 0, MaxLength() );
        std::string lResult = LeddarUtils::LtStringUtils::Utf8Encode( aValue );
        size_t lSize = ( lResult.length() > MaxLength() ? MaxLength() : lResult.length() );
        memcpy( Storage(), lResult.c_str(), lSize );
#else
        throw std::logic_error( "Do not use wstring with UTF8 on non windows platform." );
#endif
    }

    EmitSignal( LdObject::VALUE_CHANGED );
}

// *****************************************************************************
// Function: LdTextProperty::Value
//
/// \brief   Return the string value
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

std::string
LeddarCore::LdTextProperty::Value( void ) const
{
    if( mType == TYPE_UTF16 )
    {
        throw LeddarException::LtException( "Can not return string on UTF16 text property." );
    }

    return std::string( CStorage(), std::find( CStorage(), CStorage() + MaxLength(), '\0' ) );
}

// *****************************************************************************
// Function: LdTextProperty::WValue
//
/// \brief   Return the wstring value
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

std::wstring
LeddarCore::LdTextProperty::WValue( void ) const
{
    std::wstring lResult;

    if( mType == TYPE_ASCII )
    {
#ifdef _WIN32
        std::string lUtf8Str = std::string( CStorage(), std::find( CStorage(), CStorage() + MaxLength(), '\0' ) );
        lResult = LeddarUtils::LtStringUtils::Utf8Decode( lUtf8Str );
#else
        throw LeddarException::LtException( "Can not return wstring on ASCII text property - Do not use wstring on non Windows platform." );
#endif
    }
    else if( TYPE_UTF8 == mType )
    {
#ifdef _WIN32
        std::string lUtf8Str = std::string( CStorage(), std::find( CStorage(), CStorage() + MaxLength(), '\0' ) );
        lResult = LeddarUtils::LtStringUtils::Utf8Decode( lUtf8Str );
#else
        throw LeddarException::LtException( "Can not return wstring on UTF8 text property - Do not use wstring on non Windows platform." );
#endif
    }
    else
    {
        const uint16_t *lUTF16Buffer = reinterpret_cast<const uint16_t *>( CStorage() );
        uint16_t lMaxLength = MaxLength() / 2;

        for( int i = 0; i < lMaxLength; ++i )
        {
            if( lUTF16Buffer[i] == 0 )
                break;

            lResult += static_cast<wchar_t>( lUTF16Buffer[i] );
        }

    }

    return lResult;
}

// *****************************************************************************
// Function: LdTextProperty::GetStringValue
//
/// \brief   Return the string value. If the property is an UTF16,  it will be converted in UTF8.
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

std::string
LeddarCore::LdTextProperty::GetStringValue( size_t /*aIndex*/ ) const
{
    if( mType == TYPE_UTF16 )
    {
        std::wstring lWString = WValue();
        return std::string( lWString.begin(), lWString.end() );
    }

    return Value();
}
