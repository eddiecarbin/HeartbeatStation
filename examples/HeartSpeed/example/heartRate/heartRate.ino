/*!
   @file heart_test.ino
   @copyright	[DFRobot](http://www.dfrobot.com), 2016
   @copyright	GNU Lesser General Public License
   @author [jianghao](hao.jiang@dfrobot.com)
   @version  V1.0
   @date  20160-07-8
*/

/*!
   @brief Library for DFRobot's IR Position RAW_DATA
   @author [jianghao](hao.jiang@dfrobot.com)
*/
#include "HeartSpeed.h"
#include <Filter.h>

ExponentialFilter<long> ADCFilter(100, 0);
//HeartSpeed heartspeed(A0); 					///<Åbn den serielle port for at observere pulsværdi.
HeartSpeed heartspeed(A0, RAW_DATA); 		///<Åbn den serielle port mapper, observation af ECG diagram.

void setup() {
  Serial.begin(115200);
  /*!
      @brief Indstiller callback funktion.
  */
  heartspeed.setCB(mycb);
  /*!
      @brief Åben puls test.
  */
  heartspeed.begin();
}

void loop() {
delay(100);
}
/*!
      @brief Print the position result.
      @brief chose heartspeed(A1),To view ECG.
      @brief chose heartspeed(A1,RAW_DATA),See heart rate value.
*/
void mycb(uint8_t rawData, int value)
{
  if (rawData) {
    //Serial.println(value);

    ADCFilter.Filter(value);
    Serial.println( ADCFilter.Current());
    //delay(1000);
  } else {
    Serial.print("HeartRate Value = "); Serial.println(value);
  }
}
