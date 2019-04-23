// *****************************************************************************
// Module..: LeddarConnection
//
/// \file    LdInterfaceUsb.h
///
/// \brief   Implementation of the USB interface.
///
/// \author  David Levy
///
/// \since   January 2017
//
//
// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "LdConnection.h"
#include "LdConnectionInfoUsb.h"

#include <string>

namespace LeddarConnection
{
    class LdInterfaceUsb : public LdConnection
    {
    public:
        virtual void Connect( void ) override = 0;
        virtual void Disconnect( void ) override = 0;

        virtual void Read( uint8_t aEndPoint, uint8_t *aData, uint32_t aSize ) = 0;
        virtual void Write( uint8_t aEndPoint, uint8_t *aData, uint32_t aSize ) = 0;
        virtual void ControlTransfert( uint8_t aRequestType, uint8_t aRequest, uint8_t *aData, uint32_t aSize, uint16_t aTimeout = 1000 ) = 0;

    protected:
        LdInterfaceUsb( const LdConnectionInfoUsb *aConnectionInfo, LdConnection *aInterface = nullptr );
        ~LdInterfaceUsb();
        const LdConnectionInfoUsb  *mConnectionInfoUsb;
    };
}