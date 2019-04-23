// *****************************************************************************
// Module..: Leddar
//
/// \file    LdConnectionInfoEthernet.cpp
///
/// \brief   Connection information on ethernet devices.
///
/// \author  Patrick Boulay
///
/// \since   July 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdConnectionInfoEthernet.h"


// *****************************************************************************
// Function: LdConnectionInfoEthernet::LdConnectionInfoEthernet
//
/// \brief   Constructor.
///
/// \param  aIP          IP Adresse to connect
/// \param  aPort        Port of the ethernet connection
/// \param  aDescription Description of the connection
/// \param  aTimeout     Timeout in milliseconds for blocking receive calls. Default: 2000
/// \param  aUsed        (not used)
/// \param  aDisplayName Name of the connection to display
///
/// \author  Patrick Boulay
///
/// \since   October 2016
// *****************************************************************************

LeddarConnection::LdConnectionInfoEthernet::LdConnectionInfoEthernet( const std::string &aIP, const uint32_t aPort,
        const std::string &aDescription, const eConnectionType aType, const eProtocolType aProtocolType,
        const std::string &aUsed,
        uint32_t aTimeout,
        const std::string &aDisplayName ) :
    LdConnectionInfo( aType, aDisplayName ),
    mIP( aIP ),
    mPort( aPort ),
    mDescription( aDescription ),
    mTimeout( aTimeout ),
    mUsed( aUsed ),
    mProtocolType( aProtocolType ),
    mDeviceType( 0 )
{

}

// *****************************************************************************
// Function: LdConnectionInfoEthernet::~LdConnectionInfoEthernet
//
/// \brief   Destructor.
///
/// \author  Patrick Boulay
///
/// \since   October 2016
// *****************************************************************************

LeddarConnection::LdConnectionInfoEthernet::~LdConnectionInfoEthernet()
{
}
