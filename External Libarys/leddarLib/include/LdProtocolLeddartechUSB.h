// *****************************************************************************
// Module..: LeddarPrivate
//
/// \file    LdPrvProtocolLeddartechUSB.h
///
/// \brief   Class definition of LdPrvProtocolLeddartechUSB
///
/// \author  Patrick Boulay
///
/// \since   February 2017
//
// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "LdProtocolLeddarTech.h"
#include "LdInterfaceUsb.h"

namespace LeddarConnection
{
    class LdProtocolLeddartechUSB : public LdProtocolLeddarTech
    {
    public:
        enum eEndPoint
        {
            EP_CONFIG = 1,
            EP_DATA = 2
        };

        LdProtocolLeddartechUSB( const LdConnectionInfo *aConnectionInfo, LdConnection *aInterface );
        LdProtocolLeddartechUSB( const LdConnectionInfo *aConnectionInfo, LdConnection *aProtocol, eEndPoint aEndPoint );
        ~LdProtocolLeddartechUSB();

        virtual void QueryDeviceInfo( void ) override;
        virtual void ReadAnswer( void ) override;

    protected:
        virtual void Write( uint32_t aSize ) override;
        virtual void Read( uint32_t aSize ) override;

    private:
        LdInterfaceUsb *mInterfaceUSB;
        uint8_t     mEndPoint;
    };
}
