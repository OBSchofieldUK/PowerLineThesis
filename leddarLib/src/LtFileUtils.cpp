// *****************************************************************************
// Module..: LeddarTech
//
/// \file    LtFileUtils.cpp
///
/// \brief   Utilities for file manipulation
///
/// \author  Patrick Boulay
///
/// \since   March 2017
//
// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LtFileUtils.h"
#include "LtExceptions.h"

#include <fstream>

// *****************************************************************************
// Function: ReadFileToBuffer
//
/// \brief   Read a file and store it in an array of bytes.
///
/// \param   aFilename File name to read.
///
/// \return  Content of the file in an array of bytes.
///
/// \author  Patrick Boulay
///
/// \since   March 2017
// *****************************************************************************

std::vector<uint8_t>
LeddarUtils::LtFileUtils::ReadFileToBuffer( const std::string &aFilename )
{
    std::ifstream lFile;
    lFile.exceptions( std::ios_base::failbit );

    try
    {
        lFile.open( aFilename.c_str(), std::ios::binary | std::ios::ate );
    }
    catch( ... )
    {
        throw LeddarException::LtException( "File " + aFilename + " not found." );
    }

    if( lFile.is_open() )
    {
        std::streamsize lSize = lFile.tellg();

        if( lSize < 0 )
        {
            throw LeddarException::LtException( "Unable to read file " + aFilename );
        }

        lFile.seekg( 0, std::ios::beg );
        std::vector<uint8_t> lResultBuffer( static_cast<uint32_t>( lSize ) );

        if( !lFile.read( ( char * )&lResultBuffer[ 0 ], lSize ) )
        {
            throw LeddarException::LtException( "Unable to read file " + aFilename );
        }

        return lResultBuffer;
    }

    return std::vector<uint8_t>();
}