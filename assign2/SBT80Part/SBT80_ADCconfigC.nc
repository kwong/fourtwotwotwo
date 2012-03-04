/*****************************************************************************************/
/* Filename : SBT80_ADCconfigC.nc                                                        */
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




generic configuration SBT80_ADCconfigC() {
	provides {
					
	interface Read<uint16_t> as ReadADC0;
	interface Read<uint16_t> as ReadADC1;
	interface Read<uint16_t> as ReadADC2;
	interface Read<uint16_t> as ReadADC3;
	interface Read<uint16_t> as ReadADC6;
	interface Read<uint16_t> as ReadADC7;
  
	}
}
implementation {
	components new AdcReadClientC() as AdcRC0;
	components new AdcReadClientC() as AdcRC1;
	components new AdcReadClientC() as AdcRC2;
	components new AdcReadClientC() as AdcRC3;
	components new AdcReadClientC() as AdcRC6;
	components new AdcReadClientC() as AdcRC7;	
  
	ReadADC0 = AdcRC0;
	ReadADC1 = AdcRC1;
	ReadADC2 = AdcRC2;
	ReadADC3 = AdcRC3;
	ReadADC6 = AdcRC6;
	ReadADC7 = AdcRC7;

	AdcRC0.AdcConfigure -> SBT80_ADCconfigP.ADC0;
	AdcRC1.AdcConfigure -> SBT80_ADCconfigP.ADC1;
	AdcRC2.AdcConfigure -> SBT80_ADCconfigP.ADC2;
	AdcRC3.AdcConfigure -> SBT80_ADCconfigP.ADC3;
	AdcRC6.AdcConfigure -> SBT80_ADCconfigP.ADC6;
	AdcRC7.AdcConfigure -> SBT80_ADCconfigP.ADC7;
	

	components SBT80_ADCconfigP;

}
