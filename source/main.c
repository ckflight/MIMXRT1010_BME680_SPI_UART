
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1011.h"

#include "fsl_debug_console.h"
#include "fsl_lpspi.h"
#include "fsl_iomuxc.h"

#include "math.h"

#include "CK_TIME.h"
#include "CK_SPI.h"

#define UNUSED(x) ((void)(x))

typedef struct{

    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;

    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int16_t  par_p4;
    int16_t  par_p5;
    int8_t   par_p6;
    int8_t   par_p7;
    int16_t  par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;

    uint16_t par_h1;
    uint16_t par_h2;
    int8_t   par_h3;
    int8_t   par_h4;
    int8_t   par_h5;
    uint8_t  par_h6;
    int8_t   par_h7;

    int8_t  par_gh1;
    int16_t par_gh2;
    int8_t  par_gh3;

    uint8_t res_heat_range;
    int8_t  res_heat_val;
    int8_t  range_sw_err;

    int32_t t_fine;

}BMP680_s;

BMP680_s bmp680;

uint8_t readBuffer[8];

void BME680_Init(void);

void BME680_ConfigureSensor(void);

void BME680_GetCalibrationData(void);

float BME680_ReadTemperature(void);

float BME680_ReadPressure(void);

float BME680_ReadHumidity(void);

uint32_t BME680_ReadGasResistance(void);

uint8_t BME680_ReadStatus(void);

int main(void){

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    CK_TIME_Init();

    // Implement timeout for spi

    CK_SPI_Init(LPSPI1);

    BME680_Init();

    while(1) {

		uint8_t flag = BME680_ReadStatus();
		if(flag >> 7 == 1){ // New data available

			uint32_t time1 = CK_TIME_GetMicroSec();

			CK_SPI_Write(LPSPI1, 0x73, 0x10); // Set page to 1

			// Forced Mode
			uint8_t data = (0x02 << 5) | (0x05 << 2) | 0x01;
			CK_SPI_Write(LPSPI1, 0x74, data);

			// Read Temperature
			float tempereture = BME680_ReadTemperature();
			float pressure    = BME680_ReadPressure();
			float humidity    = BME680_ReadHumidity();
			uint32_t gas_res  = BME680_ReadGasResistance();

		    // Sea level HPA pressure 1013.25

		    float atmospheric = pressure / 100.0F;
		    float altitude = 44330.0 * (1.0 - pow(atmospheric / 1013.25, 0.1903));

		    uint32_t time2 = CK_TIME_GetMicroSec() - time1;

			PRINTF("Temperature: ");PRINTF("%.2f" , tempereture);PRINTF("\n");
			PRINTF("Pressure: ");PRINTF("%.2f" , pressure);PRINTF("\n");
			PRINTF("Altitude: ");PRINTF("%.2f" , altitude);PRINTF("\n");
			PRINTF("Humidity: ");PRINTF("%.2f" , humidity);PRINTF("\n");
			PRINTF("GasResistance: ");PRINTF("%d" , gas_res);PRINTF("\n");
			PRINTF("Time: ");PRINTF("%d" , time2);PRINTF("\n");
			PRINTF("\n");

			CK_TIME_DelayMilliSec(100);

		}


    }

}

void BME680_Init(void){

	PRINTF("BME680 Initialization\n");

    // In spi mode there is page 0 and 1 for registers.
    // 0x80 to 0xFF is page 0, 0x00 to 0x7F is page 1.
    // 0x60 and 0x50 are on page 0 as an exception.
    // Page is selected by writing 0 or 1 to spi_mem_page(bit4) of status register.

    // Also spi register addressing: 7 bit is used. Bit 8 is for read write indication.
    // So 0x73 register is for writing and 0xF3 when reading since bit 8 = 1 makes 0xF3

	CK_SPI_Write(LPSPI1, 0x73, 0x00); // Set page to 0

	CK_SPI_ReadMulti(LPSPI1, 0x50 | 0x80, readBuffer, 1);

	uint8_t device_id = readBuffer[0];

	if(device_id == 0x61){

		BME680_GetCalibrationData();

		CK_SPI_Write(LPSPI1, 0x73, 0x00); // Set page to 0

		CK_SPI_Write(LPSPI1, 0x60, 0xB6); // Software Reset

		CK_TIME_DelayMilliSec(100);

		BME680_ConfigureSensor();

	}




}

void BME680_ConfigureSensor(void){

	CK_SPI_Write(LPSPI1, 0x73, 0x10); // Set page to 1

	CK_SPI_Write(LPSPI1, 0x72, 0x01); // Humidity X1

	uint8_t data = (0x02 << 5) | (0x05 << 2) | 0x01;

	CK_SPI_Write(LPSPI1, 0x74, data); // Temperature X2, Pressure X16, Forced Mode

	CK_SPI_Write(LPSPI1, 0x75, 0x02 << 2); // IIR Filter Coefficient 3


}

void BME680_GetCalibrationData(void){

	uint8_t coeff_array[41];

	CK_SPI_Write(LPSPI1, 0x73, 0x00); // Set page to 0

	CK_SPI_ReadMulti(LPSPI1, 0x89 | 0x80, coeff_array, 25);

	CK_SPI_ReadMulti(LPSPI1, 0xE1 | 0x80, &coeff_array[25], 16);

    // Temperature related coefficients
    bmp680.par_t1 = (uint16_t) (coeff_array[34] << 8 | coeff_array[33]);
    bmp680.par_t2 = (int16_t) (coeff_array[2] << 8 | coeff_array[1]);
    bmp680.par_t3 = (int8_t) (coeff_array[3]);

    // Pressure related coefficients
    bmp680.par_p1 = (uint16_t) (coeff_array[6] << 8 | coeff_array[5]);
    bmp680.par_p2 = (int16_t) (coeff_array[8] << 8 | coeff_array[7]);
    bmp680.par_p3 = (int8_t) coeff_array[9];
    bmp680.par_p4 = (int16_t) (coeff_array[12] << 8 | coeff_array[11]);
    bmp680.par_p5 = (int16_t) (coeff_array[14] << 8 | coeff_array[13]);
    bmp680.par_p6 = (int8_t) (coeff_array[16]);
    bmp680.par_p7 = (int8_t) (coeff_array[15]);
    bmp680.par_p8 = (int16_t) (coeff_array[20] << 8 | coeff_array[19]);
    bmp680.par_p9 = (int16_t) (coeff_array[22] << 8 | coeff_array[21]);
    bmp680.par_p10 = (uint8_t) (coeff_array[23]);

    // Humidity related coefficients
    bmp680.par_h1 = (uint16_t) (((uint16_t) coeff_array[27] << 4) | (coeff_array[26] & 0x0F));
    bmp680.par_h2 = (uint16_t) (((uint16_t) coeff_array[25] << 4) | ((coeff_array[26]) >> 4));
    bmp680.par_h3 = (int8_t) coeff_array[28];
    bmp680.par_h4 = (int8_t) coeff_array[29];
    bmp680.par_h5 = (int8_t) coeff_array[30];
    bmp680.par_h6 = (uint8_t) coeff_array[31];
    bmp680.par_h7 = (int8_t) coeff_array[32];

    // Gas heater related coefficients
    bmp680.par_gh1 = (int8_t) coeff_array[37];
    bmp680.par_gh2 = (int16_t) (coeff_array[36] << 8 | coeff_array[35]);
    bmp680.par_gh3 = (int8_t) coeff_array[38];

    CK_SPI_Write(LPSPI1, 0x73, 0x10); // Set page to 1

    uint8_t temp_var = 0;

    temp_var = CK_SPI_Read(LPSPI1, 0x02 | 0x80);

    bmp680.res_heat_range = ((temp_var & 0x30) / 16);

    temp_var = CK_SPI_Read(LPSPI1, 0x00 | 0x80);

    bmp680.res_heat_val = (int8_t) temp_var;

    temp_var = CK_SPI_Read(LPSPI1, 0x04 | 0x80);

    bmp680.range_sw_err = ((int8_t) temp_var & (int8_t) 0xF0) / 16;


}

float BME680_ReadTemperature(void){

    CK_SPI_Write(LPSPI1, 0x73, 0x10); // Set page to 1

    uint8_t temp_msb  = CK_SPI_Read(LPSPI1, 0x22 | 0x80);
    uint8_t temp_lsb  = CK_SPI_Read(LPSPI1, 0x23 | 0x80);
    uint8_t temp_xlsb = CK_SPI_Read(LPSPI1, 0x24 | 0x80);

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int16_t calc_temp;

    uint32_t temp_raw = ((uint32_t)temp_msb << 12) | ((uint32_t)temp_lsb << 4) | ((uint32_t)temp_xlsb >> 4);

    var1 = ((int32_t) temp_raw >> 3) - ((int32_t) bmp680.par_t1 << 1);
    var2 = (var1 * (int32_t) bmp680.par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t) bmp680.par_t3 << 4)) >> 14;
    bmp680.t_fine = (int32_t) (var2 + var3);
    calc_temp = (int16_t) (((bmp680.t_fine * 5) + 128) >> 8);

    return calc_temp / 100.0f;

}

float BME680_ReadPressure(void){

	CK_SPI_Write(LPSPI1, 0x73, 0x10); // Set page to 1

    uint8_t pres_msb  = CK_SPI_Read(LPSPI1, 0x1F | 0x80);
    uint8_t pres_lsb  = CK_SPI_Read(LPSPI1, 0x20 | 0x80);
    uint8_t pres_xlsb = CK_SPI_Read(LPSPI1, 0x21 | 0x80);

    uint32_t pres_raw = ((uint32_t)pres_msb << 12) | ((uint32_t)pres_lsb << 4) | ((uint32_t)pres_xlsb >> 4);

    float var1 = 0;
    float var2 = 0;
    float var3 = 0;
    float calc_pres = 0;

    var1 = (((float)bmp680.t_fine / 2.0f) - 64000.0f);
    var2 = var1 * var1 * (((float)bmp680.par_p6) / (131072.0f));
    var2 = var2 + (var1 * ((float)bmp680.par_p5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)bmp680.par_p4) * 65536.0f);
    var1 = (((((float)bmp680.par_p3 * var1 * var1) / 16384.0f) + ((float)bmp680.par_p2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)bmp680.par_p1));
    calc_pres = (1048576.0f - ((float)pres_raw));

    // Avoid exception caused by division by zero
    if ((int)var1 != 0) {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
        var1 = (((float)bmp680.par_p9) * calc_pres * calc_pres) / 2147483648.0f;
        var2 = calc_pres * (((float)bmp680.par_p8) / 32768.0f);
        var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) * (bmp680.par_p10 / 131072.0f));
        calc_pres = (calc_pres + (var1 + var2 + var3 + ((float)bmp680.par_p7 * 128.0f)) / 16.0f);
    } else {
        calc_pres = 0;
    }

    return calc_pres;
}

float BME680_ReadHumidity(void){

	CK_SPI_Write(LPSPI1, 0x73, 0x10); // Set page to 1

    uint8_t hum_msb  = CK_SPI_Read(LPSPI1, 0x25 | 0x80);
    uint8_t hum_lsb  = CK_SPI_Read(LPSPI1, 0x26 | 0x80);
    uint16_t hum_raw = hum_msb << 8 | hum_lsb;

    float calc_hum = 0;
    float var1 = 0;
    float var2 = 0;
    float var3 = 0;
    float var4 = 0;
    float temp_comp;

    /* compensated temperature data*/
    temp_comp  = ((bmp680.t_fine) / 5120.0f);

    var1 = (float)((float)hum_raw) - (((float)bmp680.par_h1 * 16.0f) + (((float)bmp680.par_h3 / 2.0f) * temp_comp));

    var2 = var1 * ((float)(((float)bmp680.par_h2 / 262144.0f) * (1.0f + (((float)bmp680.par_h4 / 16384.0f) * temp_comp) + (((float)bmp680.par_h5 / 1048576.0f) * temp_comp * temp_comp))));

    var3 = (float) bmp680.par_h6 / 16384.0f;

    var4 = (float) bmp680.par_h7 / 2097152.0f;

    calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    if (calc_hum > 100.0f)
        calc_hum = 100.0f;
    else if (calc_hum < 0.0f)
        calc_hum = 0.0f;

    return calc_hum;
}

uint32_t BME680_ReadGasResistance(void){

	CK_SPI_Write(LPSPI1, 0x73, 0x10); // Set page to 1

    uint8_t gas_reg1  = CK_SPI_Read(LPSPI1, 0x2A | 0x80);
    uint8_t gas_reg2  = CK_SPI_Read(LPSPI1, 0x2B | 0x80);
    uint16_t gas_raw  = (gas_reg1 << 2) | (gas_reg2 >> 6);
    uint8_t gas_range = gas_reg2 & 0x0F;

    int64_t var1;
    uint64_t var2;
    int64_t var3;
    uint32_t calc_gas_res;

    // Look up table 1 for the possible gas range values
    uint32_t lookupTable1[16] = { UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
        UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
        UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
        UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647) };

    // Look up table 2 for the possible gas range values
    uint32_t lookupTable2[16] = { UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
        UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016),
        UINT32_C(8000000), UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000),
        UINT32_C(250000), UINT32_C(125000) };

    var1 = (int64_t) ((1340 + (5 * (int64_t) bmp680.range_sw_err)) * ((int64_t) lookupTable1[gas_range])) >> 16;
    var2 = (((int64_t) ((int64_t) gas_raw << 15) - (int64_t) (16777216)) + var1);
    var3 = (((int64_t) lookupTable2[gas_range] * (int64_t) var1) >> 9);
    calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

    return calc_gas_res;
}

uint8_t BME680_ReadStatus(void){

	CK_SPI_Write(LPSPI1, 0x73, 0x10); // Set page to 1

    // Bit 7 is new data indicator.
    uint8_t status_ = CK_SPI_Read(LPSPI1, 0x1D | 0x80);

    return status_;
}






























