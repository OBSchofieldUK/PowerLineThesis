// *****************************************************************************
// Module..: Leddar
//
/// \file    LdEnumProperty.h
///
/// \brief   Definition of class LdEnumProperty.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "LdProperty.h"

namespace LeddarCore
{

    /// \brief  Class for properties that have a limited set of pre-defined
    ///         values.
    class LdEnumProperty : public LdProperty
    {
    public:
        LdEnumProperty( LdProperty::eCategories aCategory, uint32_t aFeatures, uint32_t aId, uint16_t aDeviceId, uint32_t aUnitSize, bool aStoreValue = true, const std::string &aDescription = "" );

        size_t EnumSize( void ) const { return mEnumValues.size(); }
        std::string EnumText( size_t aIndex ) const { return mEnumValues[ aIndex ].second; }
        uint32_t EnumValue( size_t aIndex ) const { return mEnumValues[ aIndex ].first; }
        uint32_t ValueIndex( size_t aIndex = 0 ) const;
        uint32_t Value( size_t aIndex = 0 ) const;
        uint32_t DeviceValue( size_t aIndex = 0 ) const { return reinterpret_cast<const uint32_t *>( BackupStorage() )[aIndex]; }
        uint32_t GetKeyFromValue( const std::string &aValue );

        void SetEnumSize( size_t aSize ) { mEnumValues.clear(); mEnumValues.reserve( aSize ); }
        void AddEnumPair( uint32_t aValue, const std::string &aText );
        void ClearEnum( void );
        void SetValueIndex( size_t aArrayIndex, uint32_t aEnumIndex );
        void SetValue( size_t aIndex, uint32_t aValue );

        virtual std::string GetStringValue( size_t aIndex = 0 ) const override;
        virtual void SetStringValue( size_t aIndex, const std::string &aValue ) override;

    private:
        typedef std::pair<uint32_t, std::string> Pair;
        std::vector<Pair> mEnumValues;
        bool mStoreValue;
    };
}