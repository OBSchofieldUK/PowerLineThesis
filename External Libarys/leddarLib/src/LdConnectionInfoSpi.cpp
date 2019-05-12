// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionInfoSpi.cpp
///
/// \brief   Connection information on SPI devices.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdConnectionInfoSpi.h"
#include "LtStringUtils.h"

using namespace LeddarConnection;

// *****************************************************************************
// Function: LdConnectionInfoSpi::LdConnectionInfoSpi
//
/// \brief   Constructor.
///
/// \param  aConnectionType Type of connection
/// \param  aDisplayName    Name to display for this device.
/// \param  aAddress        Address for this SPI deveice.
/// \param  aClock          Clock speed (default: 1000 kHz).
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

LdConnectionInfoSpi::LdConnectionInfoSpi( eConnectionType aConnectionType, const std::string &aDisplayName, uint32_t aAddress, uint32_t aClock ) :
    LdConnectionInfo( aConnectionType, aDisplayName ),
    mIntAddress( aAddress ),
    mClock( aClock )
{
    SetAddress( LeddarUtils::LtStringUtils::IntToString( aAddress ) );
}

// *****************************************************************************
// Function: LdConnectionInfoSpi::~LdConnectionInfoSpi
//
/// \brief   Destructor.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

LdConnectionInfoSpi::~LdConnectionInfoSpi()
{
}
