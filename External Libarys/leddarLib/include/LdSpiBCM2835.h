// *****************************************************************************
// Module..: LeddarConnection
//
/// \file    LdSpiBCM2835.h
///
/// \brief   Implementation of the BCM2835 SPI interface on a Raspberry PI.
///          You need to uncomment the define SPI_BCM2835 in LdDefines.h to compile this class.
///
/// \author  Patrick Boulay
///
/// \since   April 2017
//
// Copyright (c) 2017 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#pragma once
#include "LdProtocolsDefines.h"
#ifdef SPI_BCM2835

#include "LdInterfaceSpi.h"
#include "LdConnectionInfo.h"

#include <vector>


namespace LeddarConnection
{

    class LdSpiBCM2835 : public LdInterfaceSpi
    {
    public:
        LdSpiBCM2835(const LdConnectionInfo* aConnectionInfo, LdConnection *aInterface = nullptr);
        virtual ~LdSpiBCM2835();

        virtual void Connect( void );
        virtual void Disconnect( void );
        virtual bool IsConnected(void);
        virtual void SetSpiConfig( eCSMode aCSMode,
                                   uint32_t aChipSelect,
                                   uint32_t aClockRate,
                                   eClockPolarity aClockPolarity,
                                   eClockPhase aClockPhase,
                                   uint32_t aBitsPerSample );

        virtual void    Transfert( uint8_t *aInputData, uint8_t *aOutputData, uint32_t aDataSize, bool aEndTransfert = false );
        virtual void    EndTransfert( void );

        virtual void    Read( uint8_t *aOutputData, uint32_t aDataSize, bool aEndTransfert = false );
        virtual void    Write( uint8_t *aOutputData, uint32_t aDataSize, bool aEndTransfert = false );

        virtual void    InitGPIO( const uint32_t& aDirection );
        virtual uint32_t ReadGPIO( const uint32_t& aPinsMask );
        virtual void    WriteGPIO( const uint32_t& aPinsMask, const uint32_t& aPinsValues );
        virtual uint8_t GetGPIOPin( eSpiPin aPin );

		virtual void    ChipSelectEnable( void );
		virtual void    ChipSelectDisable( void );
        
        static std::vector<LdConnectionInfo*> GetDeviceList( void );

    private:
        
        static void InitLib( void );
        static bool mInitialized;
        bool mIsConnected;        
    };

}

#endif