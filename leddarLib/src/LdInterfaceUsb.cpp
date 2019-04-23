// *****************************************************************************
// Module..: LeddarConnection
//
/// \file    LdInterfaceUsb.cpp
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

#include "LdInterfaceUsb.h"

// *****************************************************************************
// Function: LdInterfaceUsb::LdInterfaceUsb
//
/// \brief   Constructor.
///
/// \param   aConnectionInfo Connection information.
/// \param   aInterface      Interface for this interface (optional).
///
/// \author  David Levy
///
/// \since   January 2017
// *****************************************************************************

LeddarConnection::LdInterfaceUsb::LdInterfaceUsb( const LdConnectionInfoUsb *aConnectionInfo, LdConnection *aInterface ) :
    LdConnection( aConnectionInfo, aInterface ),
    mConnectionInfoUsb( aConnectionInfo )
{


}

// *****************************************************************************
// Function: LdInterfaceUsb::~LdInterfaceUsb
//
/// \brief   Destructor.
///
/// \author  David Levy
///
/// \since   January 2017
// *****************************************************************************
LeddarConnection::LdInterfaceUsb::~LdInterfaceUsb()
{
}