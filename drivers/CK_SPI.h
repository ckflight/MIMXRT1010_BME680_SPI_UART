#ifndef CK_SPI_H_
#define CK_SPI_H_

#include "fsl_lpspi.h"

void CK_SPI_Init(LPSPI_Type* LPSPIx);

void CK_SPI_Write(LPSPI_Type* LPSPIx, uint8_t reg, uint8_t data);

status_t CK_SPI_Write2(LPSPI_Type* LPSPIx, uint8_t reg, uint8_t data);

uint8_t CK_SPI_Read(LPSPI_Type* LPSPIx, uint8_t reg);

void CK_SPI_ReadMulti(LPSPI_Type* LPSPIx, uint8_t reg, uint8_t* readBuffer, uint16_t readSize);


#endif /* CK_SPI_H_ */
