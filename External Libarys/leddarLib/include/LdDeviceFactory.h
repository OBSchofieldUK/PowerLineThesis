// *****************************************************************************
// Module..: Leddar
//
/// \file    LdDeviceFactory.h
///
/// \brief   Factory to create devices.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once

#include "LdConnection.h"
#include "LdSensor.h"

namespace LeddarDevice
{

    class LdDeviceFactory
    {
    public:
        static LdSensor *CreateSensor( LeddarConnection::LdConnection *aConnection );
    };
}