// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionInfoSpi.h
///
/// \brief   Connection information on SPI devices.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once
#include "LdConnectionInfo.h"
#include "LtDefines.h"

namespace LeddarConnection
{
    class LdConnectionInfoSpi : public LdConnectionInfo
    {
    public:
        LdConnectionInfoSpi( eConnectionType aConnectionType, const std::string& aDisplayName, uint32_t aAddress, uint32_t aClock = 1000 );
        virtual ~LdConnectionInfoSpi();

        virtual const uint32_t& GetIntAddress( void ) const
        {
            return mIntAddress;
        }

        virtual const uint32_t& GetClock( void ) const
        {
            return mClock;
        }

        virtual void SetClock( uint32_t aClock )
        {
            mClock = aClock;
        }

    protected:
        uint32_t mIntAddress;
        uint32_t mClock;
    };
}
