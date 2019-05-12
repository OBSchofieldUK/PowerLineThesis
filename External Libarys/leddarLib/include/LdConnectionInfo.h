// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionInfo.h
///
/// \brief   Interface of the connection info retreived by the device list.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once
#include "LdProtocolsDefines.h"

#include <string>

namespace LeddarConnection
{
    class LdConnectionInfo
    {
    public:
        enum eConnectionType
        {
            CT_SPI_FTDI,
#ifdef SPI_BCM2835
            CT_SPI_BCM2835,
#endif
            CT_LIB_MODBUS,
            CT_ETHERNET_UNIVERSAL,
            CT_ETHERNET_LEDDARTECH,
            CT_USB
        };

        virtual ~LdConnectionInfo();

        virtual const std::string &GetDisplayName( void ) const {
            return mDisplayName;
        }

        virtual const std::string &GetAddress( void ) const {
            return mAddress;
        }

        virtual void SetAddress( const std::string &aAddress ) {
            mAddress = aAddress;
        }

        const eConnectionType &GetType( void ) const {
            return mType;
        }

    protected:
        LdConnectionInfo( eConnectionType aConnectionType, const std::string &aDisplayName );

        std::string mDisplayName;
        std::string mAddress;
        eConnectionType mType;

    };
}
