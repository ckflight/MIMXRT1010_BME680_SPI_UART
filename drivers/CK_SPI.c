
#include "CK_SPI.h"

#include "fsl_iomuxc.h"
#include "fsl_gpio.h"

#define CS_AUTO		0

/*
 * Read and Write uses LPSPI_MasterTransferBlocking method
 * and actually both of them are similar.
 * LPSPI_MasterTransferBlocking first writes and reads.
 * In write method i ignored results.
 * In read method it return these second read value.
 *
 */

void CK_SPI_Init(LPSPI_Type* LPSPIx){

	if(LPSPIx == LPSPI1){

		// GPIO_AD_03 is configured as LPSPI1_SDI
		// Software Input On Field: Input Path is determined by functionality
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_03_LPSPI1_SDI, 0U);

		// GPIO_AD_04 is configured as LPSPI1_SDO
		// Software Input On Field: Input Path is determined by functionality
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_04_LPSPI1_SDO, 0U);

		// GPIO_AD_06 is configured as LPSPI1_SCK
		// Software Input On Field: Input Path is determined by functionality
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_06_LPSPI1_SCK, 0U);

		#if CS_AUTO

			// GPIO_AD_05 is configured as LPSPI1_PCS0
			IOMUXC_SetPinMux(IOMUXC_GPIO_AD_05_LPSPI1_PCS0, 0U);

		#else

			// GPIO_AD_05 is configured as IO19
			IOMUXC_SetPinMux(IOMUXC_GPIO_AD_05_GPIOMUX_IO19, 0U);

		#endif

		/* GPIO_AD_03 PAD functional properties : */
		/* Slew Rate Field: Slow Slew Rate
		Drive Strength Field: R0/4
		Speed Field: fast(150MHz)
		Open Drain Enable Field: Open Drain Disabled
		Pull / Keep Enable Field: Pull/Keeper Enabled
		Pull / Keep Select Field: Keeper
		Pull Up / Down Config. Field: 100K Ohm Pull Down
		Hyst. Enable Field: Hysteresis Disabled */
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_03_LPSPI1_SDI, 0x10A0U);

		// GPIO_AD_04 PAD functional properties :
		// Slew Rate Field: Slow Slew Rate
		// Drive Strength Field: R0/4
		// Speed Field: fast(150MHz)
		// Open Drain Enable Field: Open Drain Disabled
		// Pull / Keep Enable Field: Pull/Keeper Enabled
		// Pull / Keep Select Field: Keeper
		// Pull Up / Down Config. Field: 100K Ohm Pull Down
		// Hyst. Enable Field: Hysteresis Disabled */
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_04_LPSPI1_SDO, 0x10A0U);

		// GPIO_AD_06 PAD functional properties :
		// Slew Rate Field: Slow Slew Rate
		// Drive Strength Field: R0/4
		// Speed Field: fast(150MHz)
		// Open Drain Enable Field: Open Drain Disabled
		// Pull / Keep Enable Field: Pull/Keeper Enabled
		// Pull / Keep Select Field: Keeper
		// Pull Up / Down Config. Field: 100K Ohm Pull Down
		// Hyst. Enable Field: Hysteresis Disabled
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_06_LPSPI1_SCK, 0x10A0U);

		#if CS_AUTO

			IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_05_LPSPI1_PCS0, 0x10A0U);

		#else

			IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_05_GPIOMUX_IO19, 0x70A0U);
			gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
			GPIO_PinInit(GPIO1, 19, &gpio_config);
			GPIO_PortSet(GPIO1, 1u << 19);

		#endif

	}

	// Set clock source for LPSPI1

	// Select USB1 PLL PFD0 (720 MHz) as lpspi clock source
	CLOCK_SetMux(kCLOCK_LpspiMux, 1U);

	uint32_t LPSPI_CLOCK_SOURCE_DIVIDER = 7U;
	CLOCK_SetDiv(kCLOCK_LpspiDiv, LPSPI_CLOCK_SOURCE_DIVIDER);

	lpspi_master_config_t masterConfig;


	// Master Configuration
	masterConfig.baudRate     = 10000000; // 10 MHz
	masterConfig.bitsPerFrame = 8;
	masterConfig.cpol         = kLPSPI_ClockPolarityActiveHigh;
	masterConfig.cpha         = kLPSPI_ClockPhaseFirstEdge;
	masterConfig.direction    = kLPSPI_MsbFirst;

	masterConfig.pcsToSckDelayInNanoSec        = 1000000000 / masterConfig.baudRate;
	masterConfig.lastSckToPcsDelayInNanoSec    = 1000000000 / masterConfig.baudRate;
	masterConfig.betweenTransferDelayInNanoSec = 1000000000 / masterConfig.baudRate;

	masterConfig.whichPcs           = kLPSPI_Pcs0;
	masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

	masterConfig.pinCfg        = kLPSPI_SdiInSdoOut;
	masterConfig.dataOutConfig = kLpspiDataOutRetained;

	uint32_t LPSPI_MASTER_CLK_FREQ = (CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (LPSPI_CLOCK_SOURCE_DIVIDER + 1U));
	LPSPI_MasterInit(LPSPIx, &masterConfig, LPSPI_MASTER_CLK_FREQ);

}

void CK_SPI_Write(LPSPI_Type* LPSPIx, uint8_t reg, uint8_t data){


	lpspi_transfer_t masterXfer;

	uint8_t masterTxData[2];
	masterTxData[0] = reg;
	masterTxData[1] = data;
    // Start master transfer, transfer data to slave
    masterXfer.txData   = masterTxData;
    masterXfer.rxData   = NULL;
    masterXfer.dataSize = 2;
    masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_SlaveByteSwap;

    // Both software CS drive or manual drive is working.

	//GPIO_PortClear(GPIO1, 1u << 19);
    LPSPI_MasterTransferBlocking(LPSPI1, &masterXfer);
	//GPIO_PortSet(GPIO1, 1u << 19);

}

status_t CK_SPI_Write2(LPSPI_Type* LPSPIx, uint8_t reg, uint8_t data){

	lpspi_transfer_t masterXfer;

	uint8_t masterTxData[2];
	masterTxData[0] = reg;
	masterTxData[1] = data;
    // Start master transfer, transfer data to slave
    masterXfer.txData   = masterTxData;
    masterXfer.rxData   = NULL;
    masterXfer.dataSize = 2;
    masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_SlaveByteSwap;


    // Check that LPSPI is not busy.
    if (LPSPI_GetStatusFlags(LPSPI1) & kLPSPI_ModuleBusyFlag)
    {
        return kStatus_LPSPI_Busy;
    }


    // The TX and RX FIFO sizes are always the same
    uint32_t fifoSize = LPSPI_GetRxFifoSize(LPSPI1);
    uint32_t whichPcs = (masterXfer.configFlags & LPSPI_MASTER_PCS_MASK) >> LPSPI_MASTER_PCS_SHIFT;

    bool isPcsContinuous = (bool)(masterXfer.configFlags & kLPSPI_MasterPcsContinuous);
    bool isRxMask        = false;

    LPSPI_FlushFifo(LPSPI1, true, true);
    LPSPI_ClearStatusFlags(LPSPI1, kLPSPI_AllStatusFlag);

    isRxMask = true;

    GPIO_PortClear(GPIO1, 1u << 19);


    LPSPI1->TCR = (LPSPI1->TCR & ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK)) |
                   LPSPI_TCR_CONT(isPcsContinuous) | LPSPI_TCR_CONTC(0) | LPSPI_TCR_RXMSK(isRxMask) | LPSPI_TCR_PCS(whichPcs);


    // Wait until TX FIFO is not full
    while (LPSPI_GetTxFifoCount(LPSPI1) == fifoSize);

    LPSPI_WriteData(LPSPI1, reg);

    // Wait until TX FIFO is not full
    while (LPSPI_GetTxFifoCount(LPSPI1) == fifoSize);

    LPSPI_WriteData(LPSPI1, data);


    // If no RX buffer, then transfer is not complete until transfer complete flag sets
    while (!(LPSPI_GetStatusFlags(LPSPI1) & kLPSPI_TransferCompleteFlag));


    GPIO_PortSet(GPIO1, 1u << 19);

    return kStatus_Success;


}

uint8_t CK_SPI_Read(LPSPI_Type* LPSPIx, uint8_t reg){


	lpspi_transfer_t masterXfer;

	uint8_t masterTxData[2];
	masterTxData[0] = reg;
	masterTxData[1] = 0xFF; // Send 0xFF to read

	uint8_t masterRxData[2];


    // Start master transfer, transfer data to slave
    masterXfer.txData   = masterTxData;
    masterXfer.rxData   = masterRxData;
    masterXfer.dataSize = 2;
    masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_SlaveByteSwap;

    // Both software CS drive or manual drive is working.

    //GPIO_PortClear(GPIO1, 1u << 19);
    LPSPI_MasterTransferBlocking(LPSPI1, &masterXfer);
    //GPIO_PortSet(GPIO1, 1u << 19);

    return masterRxData[1];

}

void CK_SPI_ReadMulti(LPSPI_Type* LPSPIx, uint8_t reg, uint8_t* readBuffer, uint16_t readSize){


	lpspi_transfer_t masterXfer;

	uint8_t masterTxData[readSize+1];
	uint8_t masterRxData[readSize+1];

	masterTxData[0] = reg;
	for(int i = 1; i < readSize+1; i++){
		masterTxData[i] = 0xFF; // Send 0xFF to read
	}

    // Start master transfer, transfer data to slave
    masterXfer.txData   = masterTxData;
    masterXfer.rxData   = masterRxData;
    masterXfer.dataSize = readSize+1;
    masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_SlaveByteSwap;

    // Transfer inside does not drive CS pin correctly.
    // When i implemented GPIO manual CS pin drive, it worked.

    GPIO_PortClear(GPIO1, 1u << 19);
    LPSPI_MasterTransferBlocking(LPSPI1, &masterXfer);
    GPIO_PortSet(GPIO1, 1u << 19);

    for(int i = 1; i < readSize+1; i++){
    	*readBuffer++ = masterRxData[i];
	}

}



























