/* ATM90E36 Energy Monitor Demo Application

   The MIT License (MIT)

  Copyright (c) 2016 whatnick and Ryzee

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include <ATM90E36.h>
#include <SPI.h>
#define DEBUG_PORT Serial
ATM90E36 eic(PA4);

void setup() {
  /* Initialize the serial port to host */
  DEBUG_PORT.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }
  DEBUG_PORT.println("Start ATM90E36");
  /*Initialise the ATM90E36 + SPI port */
  eic.begin_simple();
  delay(1000);
  calibrate();
}

void loop() {
  static unsigned long count = 1;
  /*Repeatedly fetch some values from the ATM90E36 */
  double voltageA, freq, voltageB, voltageC, currentA, currentB, currentC,
      power, pf, new_current, new_power;
  double pfa, pfb, pfc, pft, paa, pab, pac, pat;
  delay(100);
  int sys0 = eic.GetSysStatus0();
  int sys1 = eic.GetSysStatus1();
  int en0 = eic.GetMeterStatus0();
  int en1 = eic.GetMeterStatus1();
  Serial.print("S0:0x");
  Serial.println(sys0, BIN);
  Serial.print("S1:0x");
  Serial.println(sys1, BIN);
  Serial.println("E0:0x" + String(en0, HEX));
  Serial.println("E1:0x" + String(en1, HEX));
  voltageA = eic.GetLineVoltage(0);
  Serial.println("VA:" + String(voltageA) + "V");
  voltageB = eic.GetLineVoltage(1);
  Serial.println("VB:" + String(voltageB) + "V");
  voltageC = eic.GetLineVoltage(2);
  Serial.println("VC:" + String(voltageC) + "V");
  if (eic.testICDefaults()) {
    Serial.println("ERROR: Test IC Defaults Failed");
  }
  if (eic.calibrationError() || eic.checkOperationModeError()) {
    Serial.println("Error In Calibration, Calibrating Again");
    calibrate();
  }
  if (count++ % 5 == 0) {
    // We have done 5 loops, now we'll intentionally reset the IC and check if
    // the calibration values are retained.
    Serial.println(eic.GetValueRegister(UgainA), HEX);
    Serial.println(eic.GetValueRegister(UgainB), HEX);
    Serial.println(eic.GetValueRegister(UgainC), HEX);
    Serial.println(eic.GetValueRegister(IgainA), HEX);
    Serial.println(eic.GetValueRegister(IgainB), HEX);
    Serial.println(eic.GetValueRegister(ConfigStart), HEX);
    Serial.println(eic.GetValueRegister(CalStart), HEX);
    Serial.println(eic.GetValueRegister(HarmStart), HEX);
    Serial.println(eic.GetValueRegister(AdjStart), HEX);
    Serial.println("=====================================");
    eic.reset();
    delay(1000);
    Serial.println(eic.GetValueRegister(UgainA), HEX);
    Serial.println(eic.GetValueRegister(UgainB), HEX);
    Serial.println(eic.GetValueRegister(UgainC), HEX);
    Serial.println(eic.GetValueRegister(IgainA), HEX);
    Serial.println(eic.GetValueRegister(IgainB), HEX);
    Serial.println(eic.GetValueRegister(ConfigStart), HEX);
    Serial.println(eic.GetValueRegister(CalStart), HEX);
    Serial.println(eic.GetValueRegister(HarmStart), HEX);
    Serial.println(eic.GetValueRegister(AdjStart), HEX);
    Serial.println("================== END ===================");
  }

  delay(100);
  delay(5000);
}

void calibrate() {
  int Ugaina = 0x0005;
  int Igaina = 0x0004;
  int Uoffseta = 0x0000;
  int Ioffseta = 0x0000;
  int Ugainb = 0x0006;
  int Igainb = 0x0006;
  int Uoffsetb = 0x0000;
  int Ioffsetb = 0x0000;
  int Ugainc = 0x0008;
  int Igainc = 0x0008;
  int Uoffsetc = 0x0000;
  int Ioffsetc = 0x0000;
  int Igainn = 0x0003;

  eic.calibrateNew(Ugaina, Ugainb, Ugainc, Igaina, Igainb, Igainc, Igainn, 0x5,
                   0x5, 0x5, 0x6, 0x6, 0x6, 0x2, 0x0087);
}