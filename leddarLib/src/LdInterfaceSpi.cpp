// *****************************************************************************
// Module..: LeddarConnection
//
/// \file    LdInterfaceSpi.cpp
///
/// \brief   Interface for SPI connection.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdInterfaceSpi.h"

using namespace LeddarConnection;

// *****************************************************************************
// Function: LdInterfaceSpi::LdInterfaceSpi
//
/// \brief   Constructor.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

LdInterfaceSpi::LdInterfaceSpi( const LdConnectionInfo *aConnectionInfo, LdConnection *aInterface ):
    LdConnection( aConnectionInfo, aInterface )
{


}


// *****************************************************************************
// Function: LdInterfaceSpi::~LdInterfaceSpi
//
/// \brief   Destructor.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************

LdInterfaceSpi::~LdInterfaceSpi()
{
}

