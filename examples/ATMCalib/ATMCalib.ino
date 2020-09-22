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
#include <Embedis.h>
#include <SPI.h>
#define DEBUG_PORT Serial

Embedis embedis(DEBUG_PORT);
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
  embedisSetup();
}

void loop() {
  static unsigned long count = 1;
  /*Repeatedly fetch some values from the ATM90E36 */
  delay(100);
  int sys0 = eic.GetSysStatus0();
  int sys1 = eic.GetSysStatus1();
  int en0 = eic.GetMeterStatus0();
  int en1 = eic.GetMeterStatus1();
  Serial.print("S0:0x");
  Serial.println(sys0 & 0xFFFF, BIN);
  Serial.print("S1:0x");
  Serial.println(sys1 & 0xFFFF, BIN);
  Serial.println("E0:0x" + String(en0, HEX));
  Serial.println("E1:0x" + String(en1, HEX));
  printMeteringData();
  if (eic.testICDefaults()) {
    Serial.println("ERROR: Test IC Defaults Failed");
  }
  if (eic.calibrationError() || eic.checkOperationModeError()) {
    Serial.println("Error In Calibration, Calibrating Again");
    eic.reset();
    calibrate();
  }
  if (count++ % 5 == 0) {
    // We have done 5 loops, now we'll intentionally reset the IC and check if
    // the calibration values are retained.
    // resetIcandDisturbCalibration();
  }
  embedis.process();
  delay(100);
  delay(1000);
}
void resetIcandDisturbCalibration() {
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
// CALCULATEGAINS 240 0.83 
void embedisSetup() {
  // Add analogRead command to mirror Arduino's
  Embedis::command(F("CALCULATEGAINS"), [](Embedis* e) {
    //     if (e->argc != 2) return e->response(Embedis::ARGS_ERROR);
    //     int pin = String(e->argv[1]).toInt();
    //     e->response(':', analogRead(pin));

    GainValue result = eic.calculateGainValues(String(e->argv[1]).toInt(),
                                               String(e->argv[2]).toInt());

    Serial.println("New Gain Values");
    for (int i = 0; i < 3; i++) {
      Serial.print(result.Ugain[i], HEX);
      Serial.print(",");
      Serial.print(result.Igain[i], HEX);
      Serial.print(",  ");
    }
    // call CalibrateNew function
    Serial.println(" ");
    e->response("Calculated Gain");
  });
  Embedis::command(F("RESETIC"),
                   [](Embedis* e) { resetIcandDisturbCalibration(); });
  Embedis::command(F("CALIB"),
                   [](Embedis* e) { calibrate(); });
  
}

void printMeteringData() {
  double voltageA, freq, voltageB, voltageC, currentA, currentB, currentC,
      power, pf, new_current, new_power;
  double pfa, pfb, pfc, pft, paa, pab, pac, pat;
  double powerA, powerB, powerC;

  voltageA = eic.GetLineVoltage(0);
  voltageB = eic.GetLineVoltage(1);
  voltageC = eic.GetLineVoltage(2);

  currentA = eic.GetLineCurrent(0);
  currentB = eic.GetLineCurrent(1);
  currentC = eic.GetLineCurrent(2);

  powerA = eic.GetActivePower(0);
  powerB = eic.GetActivePower(1);
  powerC = eic.GetActivePower(2);
  power = eic.GetActivePower(3);

  Serial.print("VA:" + String(voltageA) + "V, ");
  Serial.print("VB:" + String(voltageB) + "V, ");
  Serial.println("VC:" + String(voltageC) + "V");

  Serial.print("IA:" + String(voltageA) + "A, ");
  Serial.print("IB:" + String(voltageB) + "A, ");
  Serial.println("IC:" + String(voltageC) + "A");

  Serial.print("PA:" + String(voltageA) + "W, ");
  Serial.print("PB:" + String(voltageB) + "W, ");
  Serial.print("PC:" + String(voltageC) + "W,");
  Serial.println("Total Power:" + String(voltageB) + "W");
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