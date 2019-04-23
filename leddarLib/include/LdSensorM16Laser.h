////////////////////////////////////////////////////////////////////////////////////////////////////
/// Module..: Leddar
///
/// \file   LdSensorM16Laser.h.
///
/// \brief  Class of the M16Laser sensor
///
/// \since  August 2018
///
/// Copyright (c) 2018 LeddarTech Inc. All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////
///
#pragma once

#include "LdSensorM16.h"

namespace LeddarDevice
{
    class LdSensorM16Laser : virtual public LdSensorM16
    {
    public:
        explicit LdSensorM16Laser( LeddarConnection::LdConnection *aConnection );
        ~LdSensorM16Laser();

        virtual void    GetConstants( void ) override;
        float           GetStartTraceDistance( int aValue = -1 );

    private:
        void InitProperties( void );

    };
}