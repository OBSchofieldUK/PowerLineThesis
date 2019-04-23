// *****************************************************************************
// Module..: LeddarTech
//
/// \file    LtStringUtils.h
///
/// \brief   Utilities for std::string
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "LtDefines.h"

#include <string>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <sstream>
#include <iomanip>
#include <vector>

namespace LeddarUtils
{
    namespace LtStringUtils
    {
        int32_t StringToInt( const std::string &aData, int aBase );
        uint32_t StringToUInt( const std::string &aData, int aBase );
        float StringToFloat( const std::string &aData );

        template <typename T> std::string IntToString( T aData, int aBase = 10/* = std::dec */ );

        // Left trim
        inline std::string &LeftTrim( std::string &s )
        {
            s.erase( s.begin(), std::find_if( s.begin(), s.end(), std::not1( std::ptr_fun<int, int>( std::isspace ) ) ) );
            return s;
        }

        // Right trim
        inline std::string &RightTrim( std::string &s )
        {
            s.erase( std::find_if( s.rbegin(), s.rend(), std::not1( std::ptr_fun<int, int>( std::isspace ) ) ).base(), s.end() );
            return s;
        }

        // Trim in both side
        inline std::string &Trim( std::string &s )
        {
            return LeftTrim( RightTrim( s ) );
        }

        // Replace all occurences of a character by another one
        inline void Replace( std::string &aInput, char aCharToReplace, char aReplacementChar )
        {
            std::replace( aInput.begin(), aInput.end(), aCharToReplace, aReplacementChar );
        }

        inline std::string ToUpper( std::string aValue )
        {
            std::transform( aValue.begin(), aValue.end(), aValue.begin(), ::toupper );
            return aValue;
        }

        inline std::string ToLower( std::string aValue )
        {
            std::transform( aValue.begin(), aValue.end(), aValue.begin(), ::tolower );
            return aValue;
        }

        std::vector<std::string> Split( const std::string &aLine, char aSeparator = ' ' );

        void HexStringToByteArray( const std::string &aHexString, uint8_t *aHexByte );
        std::string ByteArrayToHexString( const uint8_t *aHexByte, uint32_t aLength );

        uint32_t StringToIp4Addr( const std::string &aIpAddrStr );
        std::string Ip4AddrToString( uint32_t aIpAddr );
        void Ip4PortStringToValues( const std::string &aIp4Port, std::string *aIp, uint16_t *aPort );
#ifdef _WIN32
        std::string Utf8Encode( const std::wstring &lWstr );
        std::wstring Utf8Decode( const std::string &lStr );
#endif

        // *****************************************************************************
        // Function: LtStringUtils::IntToString
        //
        /// \brief   Convert int to string
        /// \brief   Note : negative int8 in base 16 are displayed as int16
        ///
        /// \param   aData  Int to convert.
        /// \param   aBase  Number base of the int
        ///
        /// \author  Patrick Boulay
        ///
        /// \since   March 2016
        // *****************************************************************************
        template <typename T> std::string
        IntToString( T aData, int aBase )
        {
            std::stringstream lResult;
            lResult << ( aBase == 16 ? "0x" : "" ) << std::setbase( aBase ) << aData;
            return lResult.str();
        }
        template <> inline std::string
        IntToString<uint8_t>( uint8_t aData, int aBase )
        {
            return IntToString( static_cast<uint16_t>( aData ), aBase );
        }
        template <> inline std::string
        IntToString<int8_t>( int8_t aData, int aBase )
        {
            return IntToString( static_cast<int16_t>( aData ), aBase );
        }
    }
}
