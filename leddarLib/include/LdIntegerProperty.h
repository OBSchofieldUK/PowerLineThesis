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

#pragma once

#include "LdProperty.h"

namespace LeddarCore
{
    class LdIntegerProperty : public LdProperty
    {
    public:
        LdIntegerProperty( LdProperty::eCategories aCategory, uint32_t aFeatures, uint32_t aId,
                           uint16_t aDeviceId, uint32_t aUnitSize, const std::string &aDescription = "", const bool aSigned = false );

        int64_t MinValue( void ) const { return mMinValue; }
        int64_t MaxValue( void ) const { return mMaxValue; }
        int64_t Value( size_t aIndex = 0 ) const;
        template <class T> T ValueT( size_t aIndex = 0 ) const;
        int32_t DeviceValue( size_t aIndex = 0 ) const { return reinterpret_cast<const int32_t *>( BackupStorage() )[ aIndex ]; }

        void SetLimits( int64_t aMin, int64_t aMax );
        void SetValue( size_t aIndex, int64_t aValue );

        virtual std::string GetStringValue( size_t aIndex ) const override;
        virtual void SetStringValue( size_t aIndex, const std::string &aValue ) override;
        void SetStringValue( size_t aIndex, const std::string &aValue, uint8_t aBase );

    private:
        int64_t mMinValue, mMaxValue;
        bool mSigned;
    };
}