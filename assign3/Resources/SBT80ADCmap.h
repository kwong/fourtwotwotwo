/*****************************************************************************************/
/* Filename : SBT80ADCmap.h                                                              */
/*****************************************************************************************/
/*                                                                                       */
/* This code is intended for the EasySen SBT80 Sensor board along with TelosB            */
/* family of wireless motes. It samples the 8 sensor channels in the following order:    */
/* Visual Light (VL), Microphone (MIC), Infrared (IR), Temperature (TEMP),               */ 
/* Acceleration over x axis (ACCx), Acceleration over y axis (ACCy),                     */
/* Magnetic field over x axis (MGx), Magnetic field over y axis (MGy).                   */
/* Sampling is performed over each ADC channel individually.                             */
/*                                                                                       */
/* For further details, please refer to README.txt and                                   */ 
/* Guide_to_TinyOS20_code_for_EasySen_SensorBoards.doc                                   */
/*                                                                                       */
/* Disclaimer: Easysen does not take on any liability for the use of this code.          */
/* This code is not designed for use in critical or life support systems                 */
/* where failure to perform affects safety or effectiveness or results in                */
/* any personal injury to the user. This code is available for free downloading          */
/* at www.easysen.com, and not for sale.                                                 */ 
/*                                                                                       */
/*****************************************************************************************/


#include "Msp430Adc12.h"


const msp430adc12_channel_config_t config_ADC0 = {
		inch:	INPUT_CHANNEL_A0, // 0->VL*, 1-> MIC*, 2->IR*, 3->TEMP, 6->ACC_x, 7-> ACC_y
		sref: REFERENCE_VREFplus_AVss,
		ref2_5v: REFVOLT_LEVEL_NONE,
		adc12ssel: SHT_SOURCE_ACLK,
		adc12div:	SHT_CLOCK_DIV_1,
		sht: SAMPLE_HOLD_4_CYCLES,
		sampcon_ssel: SAMPCON_SOURCE_SMCLK,
		sampcon_id: SAMPCON_CLOCK_DIV_1
		};
  
const msp430adc12_channel_config_t config_ADC1 = {
		inch:	INPUT_CHANNEL_A1, // 0->VL*, 1-> MIC*, 2->IR*, 3->TEMP, 6->ACC_x, 7-> ACC_y
		sref: REFERENCE_VREFplus_AVss,
		ref2_5v: REFVOLT_LEVEL_2_5,
		adc12ssel: SHT_SOURCE_ACLK,
		adc12div:	SHT_CLOCK_DIV_1,
		sht: SAMPLE_HOLD_4_CYCLES,
		sampcon_ssel: SAMPCON_SOURCE_SMCLK,
		sampcon_id: SAMPCON_CLOCK_DIV_1
		};

const msp430adc12_channel_config_t config_A2 = {
		inch:	INPUT_CHANNEL_A2, // 0->VL*, 1-> MIC*, 2->IR*, 3->TEMP, 6->ACC_x, 7-> ACC_y
		sref: REFERENCE_VREFplus_AVss,
		ref2_5v: REFVOLT_LEVEL_2_5,
		adc12ssel: SHT_SOURCE_ACLK,
		adc12div:	SHT_CLOCK_DIV_1,
		sht: SAMPLE_HOLD_4_CYCLES,
		sampcon_ssel: SAMPCON_SOURCE_SMCLK,
		sampcon_id: SAMPCON_CLOCK_DIV_1
		};

const msp430adc12_channel_config_t config_A3 = {
		inch:	INPUT_CHANNEL_A3, // 0->VL*, 1-> MIC*, 2->IR*, 3->TEMP, 6->ACC_x, 7-> ACC_y
		sref: REFERENCE_AVcc_AVss,
		ref2_5v: REFVOLT_LEVEL_NONE,
		adc12ssel: SHT_SOURCE_ACLK,
		adc12div:	SHT_CLOCK_DIV_1,
		sht: SAMPLE_HOLD_4_CYCLES,
		sampcon_ssel: SAMPCON_SOURCE_SMCLK,
		sampcon_id: SAMPCON_CLOCK_DIV_1
		};

const msp430adc12_channel_config_t config_A6 = {
		inch:	INPUT_CHANNEL_A6, // 0->VL*, 1-> MIC*, 2->IR*, 3->TEMP, 6->ACC_x, 7-> ACC_y
		sref: REFERENCE_VREFplus_AVss,
		ref2_5v: REFVOLT_LEVEL_2_5,
		adc12ssel: SHT_SOURCE_ACLK,
		adc12div:	SHT_CLOCK_DIV_1,
		sht: SAMPLE_HOLD_4_CYCLES,
		sampcon_ssel: SAMPCON_SOURCE_SMCLK,
		sampcon_id: SAMPCON_CLOCK_DIV_1
		};

const msp430adc12_channel_config_t config_A7 = {
		inch:	INPUT_CHANNEL_A7, // 0->VL*, 1-> MIC*, 2->IR*, 3->TEMP, 6->ACC_x, 7-> ACC_y
		sref: REFERENCE_VREFplus_AVss,
		ref2_5v: REFVOLT_LEVEL_2_5,
		adc12ssel: SHT_SOURCE_ACLK,
		adc12div:	SHT_CLOCK_DIV_1,
		sht: SAMPLE_HOLD_4_CYCLES,
		sampcon_ssel: SAMPCON_SOURCE_SMCLK,
		sampcon_id: SAMPCON_CLOCK_DIV_1
		};




