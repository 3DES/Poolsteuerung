/***********************************
 * Simple pool control for arduino nano
 *
 * 1-wire support only for adjustment
 * During startup it will search for such a sensor and if exist it will print its current value in state machine, that's all!
 * The values can be used to calculate proper adjustment values for the 5k NTCs
 * onewire: www.netzmafia.de/skripten/hardware/Adruino/Onewire/index.html
 *
 *
 * LEDs and meaning
 *    heat | match | cool |    situation
 *    -----+-------+------+---------------------
 *      *  |       |      | water too cold, roof warmer,          pump running,     water will be heated
 *      *  |   *   |      | water too cold, roof not warm enough, pump not running
 *         |       |  *   | water too hot,  roof colder,          pump running,     water will be cooled down
 *         |   *   |  *   | water too hot,  roof not cold enough, pump not running
 *
 *
 *      voltage divider                potentiometer                                 LEDs                                          pump + heater valve                                  1-wire sensors
 *        roof/water                     temp set
 *
 *            5V                            5V                                    1k5                                                 VCC         ~24V         L(230V)                    VCC
 *            +                              +                                    ___    LED                                           +            +             +                        +
 *            |                              |                            VCC +--|___|---|>|-----+ uC                                  |            |             |                        |
 *            |                              |                                            \\                                     .-----o            |             |                        o-----.
 *           .-.                            .-.                                                                                  |     |            |             |                        |     |
 *           | | 6k8                    1k5 | |                                                                                  -    _|_           o             o                       .-.    |
 *           | |                            | |                                                                                  ^   |_/_|- - - - -/- - - - - - -/                    4k7 | |    |    DS18B20
 *           '-'                            '-'                                                                                  |     |          /             /                         | |    |    ______
 *            |                              |                                                                                   '-----o        o/  o         o/  o                       '-'    '---|3 VDD |
 *   uC +-----o---------.                    |                                                                      10k                |        |   |         |   |                        |         |      |
 *            |         |                   .-.                                                                     ___              |/    open |   | close       |                  uC +--o---------|2 DQ  |
 *           .-.        |               10k | |<--o----o----+ uC                                            uC +---|___|----o-----o--|          |   |             |                                  |      |
 *           | | 5k    --- 10n              | |   |    |                                                                    |     |  |>         |   |             |                     .------------|1_GND_|
 *           | | NTC   ---                  '-'   |    |                                                                    o    .-.   |         \ /              |                     |
 *           '-'        |                    |    |    |                                                                   /     | |2k |         .-.              |                    ===
 *            |         |                    o----'    |                                                      manual |-^--/      | |   |        /   \             |                    GND
 *            o---------'                    |        ---  10n                                                           /  o    '-'   |       |  M  |            |
 *            |                             .-.       ---                                                                   |     |    |        \   /             |
 *            |                         460 | |<--.    |                                                                    '-----o----'         '-'              |
 *           ===                            | |   |    |                                                                          |               |               |
 *           GND                            '-'   |    |                                                                          |               |               |
 *                                           |    |    |                                                                         ===              +               +
 *                                           o----'    |                                                                         GND            ~24V            pump
 *                                           |         |
 *                                           o---------'
 *                                           |
 *                                          .-.   
 *                                      1k2 | |   
 *                                          | |   
 *                                          '-'
 *                                           |
 *                                          ===
 *                                          GND
 *
 *
 *
 *   Arduino NANO
 *
 *    +-----------------------------+
 *    |o o o o o o o o o o o o o o o|
 *    |                         ISP |
 *    +-+                  GND  o o | -RESET (30)
 *    | | USB             MOSI  o o | SCK    (20) [PB1/PCINT1/SCK]                  MOSI (21) [PB2/PCINT2/MOSI]
 *    +-+                   5V  o x | MISO   (22) [PB3/PCINT3/MISO]
 *    |                             |
 *    |o o o o o o o o o o o o o o o|
 *    +-----------------------------+
 *
 *
 *
 *
 *  small PCB from sunnypool with potentiometer
 *
 *      VCC (5V)
 *  (3) -----------------------------------.
 *                                         |
 *      freez   LED red      ___           |
 *  (6) ---------|<|--------|___|----------o
 *               //          1k5           |
 *      cool    LED red      ___           |
 *  (5) ---------|<|--------|___|----------o
 *               //          1k5           |
 *      match   LED green    ___           |
 *  (2) ---------|<|--------|___|----------o
 *               //          1k5           |
 *      heat    LED red      ___           |
 *  (4) ---------|<|--------|___|----------o-----------------------------.
 *               //          1k5           |                             |
 *                                         |                             |
 *                                        .-.                            |+
 *                                        | | 1k5                       ===  47uF
 *                                        | |                            |
 *                                        '-'                            |
 *                                         |                             |
 *                                         o-------------.               |
 *                                         |             |               |
 *                                        .-.            |               |
 *  (8) --------------------------------->| | 10k       '-,              |
 *                                        | | temp set   ^  Z diode      |
 *                                        '-'            |               |
 *                                         |             |               |
 *                                         o-------------'               |
 *                                         |                             |
 *                                        .-.                            |
 *                                460 Ohm | |<--.                        |
 *                                        | |                            |
 *                                        '-'  (*)                       |
 *      (*) no need for this               |                             |
 *          potentiometer                  o----'                        |
 *          so we cut it and use           |                             |
 *          it as simple resistor         .-.                            |
 *                                        | | 1k2                        |
 *                                        | |                            |
 *                                        '-'                            |
 *                                         |                             |
 *                                         |                             |
 *                     manual              |                             |
 *                   /o--------------------o                             |
 *                  /                      |                             |
 *  (1) ----------o/  o--                  |                             |
 *                     auto                |                             |
 *                                         |                             |
 *                                         |                             |
 *      GND                                |                             |
 *  (7) -----------------------------------o-----------------------------'
 *
 *
 *
 *
 */


#include <OneWire.h>


// Arduino nano pinning
#define D0   0              // RX
#define D1   1              // TX
#define D2   2              // one wire bus
#define D3   3              // pump ON + heater valve closed (closed means water is pumped to the roof)
#define D4   4              // switchable measurement power supply
#define D5   5              // 
#define D6   6              // 
#define D7   7              // LED red   "heizen"
#define D8   8              // LED red   "Solltemperatur erreicht"
#define D9   9              // LED green "kühlen"
#define D10 10              // LED red   "Frostgefahr"
#define D11 11              //
#define D12 12              //
#define D13 13              // onboard LED

// #define D14 14              // also A0 in case of ADC
// #define D15 15              // also A1 in case of ADC
// #define D16 16              // also A2 in case of ADC
// #define D17 17              // also A3 in case of ADC
// #define D18 18              // also A4 in case of ADC
// #define D19 19              // also A5 in case of ADC

#define A0  0               // roof   temperature
#define A1  1               // wather temperature
#define A2  2               // set    temperature
#define A3  3               //
#define A4  4               //
#define A5  5               //
#define A6  6               //
#define A7  7               //


uint16_t lastError = 0;    // to store any error for blinken lights if any happened


// DS18B20 variables
bool     ds18B20Found = false;  // DS18B20 sensor found during startup?
uint8_t  ds18B20Address[8];     // address of found DS18B20 sensor
uint8_t  ds18B20Data[12];       // to collect received data
uint16_t ds18B20Value;          // lastly measured value (in °C*100, so there are two decimals contained)


/**
 * some constant values
 */
enum {
  eTempHysteresis   = 3,    // don't change pump settings within +/- hysteresis range °C
  eTempFrost        = 5,    // show frost when this temp or less has been reached

  eTempSamples      = 6,    // amount of temp samples (must be 2^n+2 because of sort and median algorithm!)
  eStateMedian      = eTempSamples,
  eStateConversion,
  eStateAction,
  eStateOneWire,
  eStatePrint,
};


/**
 * uniq error values
 */
typedef enum {
  eError_none = 0,
  eError_0001 = 0x0001,     // array length not as expected
  eError_0002 = 0x0002,     // unexpected state reached
  eError_0003 = 0x0003,     // given ADC value is smaller than given lowTempDegree  and sensor is NTC
  eError_0004 = 0x0004,     // given ADC value is larger  than given highTempDegree and sensor is NTC
  eError_0005 = 0x0005,     // given ADC value is larger  than given lowTempDegree  and sensor is PTC
  eError_0006 = 0x0006,     // given ADC value is smaller than given highTempDegree and sensor is PTC
} eError_mt;


/**
 * IOs used for ADC and digital IOs
 */
enum {
  eTempPinRoof    = A0,     // pin for roof temp sensor
  eTempPinWater   = A1,     // pin for water temp sensor
  eTempPinSet     = A2,     // pin for set temp
                            
  eOutPinOneWire  = D2,     // one wire pin (to setup temperature values)
  eOutPinPump     = D3,     // pin for pump relais
  eOutPinMeassure = D4,     // pin to supply NTCs during measurement (if they are supplied all the time they get warm by themselves!)
                            
  eLedPinHeat     = D7,     // LED pin for heating
  eLedPinMatch    = D8,     // LED pin if set temperature reached
  eLedPinCool     = D9,     // LED pin for cooling
  eLedPinFrost    = D10,    // LED pin at risk of frost
                  
  eLedPinOnBoard  = D13,    // Arduino nano onboard LED
};


// latch to pin matrix
enum {
  eLedOff     = 0x00,        // OFF usually not used

  eLedHeat    = 0x01 << 0,   // water needs to be heated
  eLedMatch   = 0x01 << 1,   // water temperature matches set temperatur
  eLedCool    = 0x01 << 2,   // water needs to be cooled down
  eLedFrost   = 0x01 << 3,   // ON in case temp of roof is less than or equal to eTempFrost since then there is any risk that water freezes!
  eLedOnBoard = 0x01 << 4,   // on board LED blinks as alive signal
  eOutPump    = 0x01 << 5,   // ON in case pump is running and heater valve has to be closed (= heating/cooling)
};
//                enum Bits:  eLedHeat     eLedMatch     eLedCool     eLedFrost     eLedOnBoard     eOutPump
const uint8_t pins[]       = {eLedPinHeat, eLedPinMatch, eLedPinCool, eLedPinFrost, eLedPinOnBoard, eOutPinPump};          // order should be identical with previous enum!!!
const uint8_t pinsActive[] = {LOW,         LOW,          LOW,         LOW,          HIGH,           HIGH};                 // value to set refering output to ON


// one wire object
OneWire ds1820(eOutPinOneWire);


#define MEDIAN(low,high)     ((low) + (((high) - (low)) / 2))


/**
 * "default" ADC values for different temperatures
 *                                  10°C            15°C                                    20°C   25°C   30°C  x35°C   40°C  x45°C   50°C    zero_terminated
 */
enum {
  // resistor values for 5k NTC
  e5kNtc10Degree = 9952U,
  e5kNtc20Degree = 6246U,
  e5kNtc25Degree = 5000U,
  e5kNtc30Degree = 4028U,
  e5kNtc40Degree = 2662U,
  e5kNtc50Degree = 1800U,

  e5kNtc15Degree = MEDIAN(e5kNtc10Degree, e5kNtc20Degree),
  e5kNtc35Degree = MEDIAN(e5kNtc30Degree, e5kNtc40Degree),
  e5kNtc45Degree = MEDIAN(e5kNtc40Degree, e5kNtc50Degree),
};
const uint16_t resistor5kNtcValues[]  = { e5kNtc10Degree, e5kNtc15Degree, e5kNtc20Degree, e5kNtc25Degree, e5kNtc30Degree, e5kNtc35Degree, e5kNtc40Degree, e5kNtc45Degree, e5kNtc50Degree, 0 };

#define ROOF_TEMP_CORRECTOR(x)   (x * 10 / 100)      // valid for 5k NTC placed on roof                 // @todo Faktor stimmt noch garnicht, muß erst ermittelt werden!!!
const uint16_t adcRoofTempValues[]    = { ROOF_TEMP_CORRECTOR(e5kNtc10Degree), ROOF_TEMP_CORRECTOR(e5kNtc15Degree), ROOF_TEMP_CORRECTOR(e5kNtc20Degree), ROOF_TEMP_CORRECTOR(e5kNtc25Degree), ROOF_TEMP_CORRECTOR(e5kNtc30Degree), ROOF_TEMP_CORRECTOR(e5kNtc35Degree), ROOF_TEMP_CORRECTOR(e5kNtc40Degree), ROOF_TEMP_CORRECTOR(e5kNtc45Degree), ROOF_TEMP_CORRECTOR(e5kNtc50Degree), 0 };

#define WATER_TEMP_CORRECTOR(x)  (x * 11 / 100)      // valid for 5k NTC placed into water              // @todo Faktor stimmt noch garnicht, muß erst ermittelt werden!!!
const uint16_t adcWaterTempValues[]   = { ROOF_TEMP_CORRECTOR(e5kNtc10Degree), ROOF_TEMP_CORRECTOR(e5kNtc15Degree), ROOF_TEMP_CORRECTOR(e5kNtc20Degree), ROOF_TEMP_CORRECTOR(e5kNtc25Degree), ROOF_TEMP_CORRECTOR(e5kNtc30Degree), ROOF_TEMP_CORRECTOR(e5kNtc35Degree), ROOF_TEMP_CORRECTOR(e5kNtc40Degree), ROOF_TEMP_CORRECTOR(e5kNtc45Degree), ROOF_TEMP_CORRECTOR(e5kNtc50Degree), 0 };


/**
 * meassured values for potentiometer
 * (*) = calculated                           (*)                                        (*)    (*)    (*)  zero_terminated
 *                                   0xDF  0x127  0x16F  0x1BB  0x200  0x257  0x2AC  0x301  0x365
 */
const uint16_t potiTempValues[] = {   223,   295,   367,   443,   512,   599,   684,   769,   854,  0 };


/**
 * assert function to check something and set error if necessary
 *
 * @successful      true in case test was OK, false if test failed
 * @error           uniq error number to be stored
 */
static inline void assert(bool successful, eError_mt error) {
  if (!successful) {
    Serial.print(F("Error: "));
    Serial.println(error);
    lastError = error;
  }
}


/**
 * Sorts given array in a simple way (ignoring largest and smallest element for EMV reasons):
 *   smalles element will be put at [length - 2] (and ignored)
 *   biggest element will be put at [length - 1] (and ignored)
 *   median value    will be calculated from the rest and returned
 *
 * @ptr         adress of uint16_t array to be "simply" sorted
 * @length      elements in array
 *
 * @result      calculated median ignoring largest and smallest element
 */
uint16_t simpleSort(uint16_t ptr[], uint8_t length) {
  assert(length == 6, eError_0001);   // ensure proper array length

  uint16_t temp;

  // search for smallest and largest element and put them into the last two array positions
  for (uint8_t loop = 0; loop < length - 2; loop++) {
    // current element smaller than current minimum?
    if (ptr[loop] < ptr[length - 2]) {
      temp = ptr[length - 2];
      ptr[length - 2] = ptr[loop];
      ptr[loop] = temp;
    }

    // current element larger than current maximum?
    if (ptr[loop] > ptr[length - 1]) {
      temp = ptr[length - 1];
      ptr[length - 1] = ptr[loop];
      ptr[loop] = temp;
    }
  }

  // calculate median of the first 4 elements and ignore smallest and largest one (for EMV reasons)
  temp = 0;
  for (uint8_t loop = 0; loop < length - 2; loop++) {
    temp += ptr[loop];
  }
  temp >>= 2;   // 4 elements added so divide them by 4 to get the median value

  return temp;
}


/**
 * Calculates degree out of given ADC and reference values
 * with linear interpolation between the two given values
 *
 * @adcValue            meassured ADC value
 * @lowTempAdc          previous known ADC value (can be smaller or larger than @highTempAdc to support PTC as well as NTC)
 * @highTempAdc         next     known ADC value
 * @lowTempDegree       degree of next smaller known ADC value (must be smaller or equal than @lowTempDegree!)
 * @highTempDegree      degree of next larger known ADC value  (must be larger  or equal than @lowTempDegree!)
 *
 * @return              degree calculated from the given information
 */
uint8_t tempCalculation(uint16_t adcValue, uint16_t lowTempDegree, uint16_t highTempDegree, uint16_t lowTempAdc, uint16_t highTempAdc) {
  uint8_t result;
  uint16_t degreeDelta = highTempDegree - lowTempDegree;

  Serial.print(F("Calculate: degreeDelta: "));
  Serial.print(degreeDelta, DEC);

  if (adcValue == lowTempAdc) {
    result = lowTempDegree;           // match with low temp ADC value found
  }
  else
  if (adcValue == highTempAdc) {
    result = highTempDegree;          // match with high temp ADC value found
  }
  else {
    uint16_t adcDelta;

    // PTC or NTC
    if (highTempAdc > lowTempAdc) {
      assert(adcValue >= lowTempAdc,  eError_0003);     // ensure given ADC value is in correct range
      assert(adcValue <= highTempAdc, eError_0004);     // ensure given ADC value is in correct range

      // PTC support
      adcDelta = highTempAdc - lowTempAdc;
      adcValue -= lowTempAdc;     // normalize ADC value [0..highTempAdc]
    }
    else {
      assert(adcValue <= lowTempAdc,  eError_0005);     // ensure given ADC value is in correct range
      assert(adcValue >= highTempAdc, eError_0006);     // ensure given ADC value is in correct range

      // NTC support
      adcDelta = lowTempAdc - highTempAdc;
      adcValue -= highTempAdc;    // normalize ADC value [0..highTempAdc]
    }

    result = lowTempDegree + (uint8_t)((((uint32_t)degreeDelta) * adcValue) / adcDelta);      // uint32_t is necessary to prevent overflows... '*' before '/' is necessary to reduce rounding errors

    Serial.print(F(" / adcValue: "));
    Serial.print(adcValue, HEX);
    Serial.print(F(" / adcDelta: "));
    Serial.print(adcDelta, HEX);
  }

  Serial.print(F(" / result degree: "));
  Serial.print(result, DEC);
  Serial.print(F("\n"));

  return result;
}

/**
 * Converts ADC value into temperature value
 *
 * @adcValue   ADC value to be converted
 * @tempArray  array containing temperature values for 10, 15, 20, 25, 30, 35, 40, 45 and 50°C followed by a ZERO as terminator
 *
 * @return     temperature equivalent in °C
 */
uint8_t tempConversion(uint16_t adcValue, const uint16_t * const tempArray) {
  uint8_t  lowIndex  = 0;       // index of ADC value smaller than adcValue content
  uint8_t  highIndex = 0;       // index of ADC value larger  than adcValue content

  uint16_t value     = tempArray[0];            // read first temp array entry
  uint16_t nextValue = tempArray[1];            // need to read second value from array to decide type of temp sensor (NTC or PTC)
  bool     PTC       = (value > nextValue);     // if first value is larger than second one it must be a PTC, otherwise it's an NTC

  Serial.print(F("Convert: "));
  Serial.print(PTC ? F("PTC") : F("NTC"));
  Serial.print(F(" / adc: "));
  Serial.print(adcValue, HEX);

  // find temperature range
  if (adcValue <= value) {
    // if temperature is less than lowest value in array take lowest value (deeper temperatures are not from interest!)
    adcValue = value;

    // low- and highIndex stay unchanged!
    // lowIndex  = 0;
    // highIndex = 0;
  }
  else {
    nextValue = value;      // needs to be reset, otherwise following loop would stop too early!

    //     [-------------PTC---------------]    [-------------NTC--------------]     [-end of array-]
    while (((PTC  && (adcValue < nextValue)) || (!PTC && (adcValue > nextValue))) && (nextValue != 0)) {
      highIndex++;
      value = nextValue;
      nextValue = tempArray[highIndex];
    }

    // end of array reached?
    if (nextValue == 0) {
      // if end of temperature array reached measured ADC is too large and must be set to last array value
      adcValue = value;

      // set indices back to last valid table entry
      highIndex--;
      lowIndex = highIndex;
    }
    else {
      // highIndex points to next larger/smaller element, so lowIndex must point to predecessor
      lowIndex = highIndex - 1;
    }
  }

  Serial.print(F(" / adc new: "));
  Serial.print(adcValue, HEX);
  Serial.print(F(" / lowIndex: "));
  Serial.print(lowIndex, DEC);
  Serial.print(F(" / highIndex: "));
  Serial.print(highIndex, DEC);
  Serial.print(F(" / lowValue: "));
  Serial.print(value, HEX);
  Serial.print(F(" / nextValue: "));
  Serial.print(nextValue, HEX);
  Serial.print(F("\n"));

  return tempCalculation(adcValue, 10 + (5 * lowIndex), 10 + (5 * highIndex), value, nextValue);
}

/**
 * setup function
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           //  setup serial

  // initialize outputs
  pinMode(eOutPinPump,     OUTPUT);
  pinMode(eLedPinHeat,     OUTPUT);
  pinMode(eOutPinMeassure, OUTPUT);  

  // initialize LEDs ports
  pinMode(eLedPinMatch,    OUTPUT);
  pinMode(eLedPinCool,     OUTPUT);
  pinMode(eLedPinFrost,    OUTPUT);
  pinMode(eLedPinOnBoard,  OUTPUT);


  // search for a 1-wire sensor (only one supporetd for temp sensor adjustment)
  ds18B20Found = ds1820.search(ds18B20Address);
  if (ds18B20Found) {
    Serial.print(F("DS18B20 found: "));

    // print found address
    for (uint8_t index = 0; index < sizeof(ds18B20Address); index++) {
      if (ds18B20Address[index] < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(ds18B20Address[index], HEX);
      Serial.print(F(" "));
    }
    Serial.print(F("\n"));

    // check CRC of message
    if (OneWire::crc8(ds18B20Address, 7) != ds18B20Address[7]) {
      Serial.print(F("CRC invalid, ignore it!\n"));
      ds18B20Found = false;
    }

    // ensure sensor type is a DS18B20
    if (ds18B20Address[0] != 0x28) {
      Serial.print(F("not a DS18B20 sensor: [0]="));
      Serial.print(ds18B20Address[0], HEX);
      ds18B20Found = false;
    }
  }
  else {
    Serial.print(F("no DS18B20 found\n"));
  }
}


/**
 * loop function (worker thread)
 *
 * state machine:
 *  1..n: measure current ADC values
 *  n+1:  remove highest and lowest value, then create median out of the rest
 *  n+2:  decide new pump and LED states, then set referring outputs
 *  n+3:  print current values, pump and LED states to serial terminal
 */
void loop() {
  static int16_t state = 0;             // current state machine state

  static uint16_t tempRoofArray[eTempSamples];      // samples of roof temp sensor
  static uint16_t tempRoofAdc;                      // roof temp ADC value
  static uint8_t  tempRoof;                         // resulting roof temp value

  static uint16_t tempWaterArray[eTempSamples];     // samples of water temp sensor
  static uint16_t tempWaterAdc;                     // water temp ADC value
  static uint8_t  tempWater;                        // resulting water temp value

  static uint16_t tempSetArray[eTempSamples];       // samples of set temperature potentiometer
  static uint16_t tempSetAdc;                       // set temp ADC value
  static uint8_t  tempSet;                          // resulting set temp value

  static bool     pump       = false;               // current pump state
  static uint8_t  portStates = 0;                   // current LEDs states

  if (state < eTempSamples) {
    // sample phases

    // switch power supply for measurement resistors ON
    digitalWrite(eOutPinMeassure, HIGH);
    
    // meassure
    tempRoofArray[state]  = analogRead(eTempPinRoof);
    tempWaterArray[state] = analogRead(eTempPinWater);
    tempSetArray[state]   = analogRead(eTempPinSet);
  }
  else
  switch (state) {
    case eStateMedian:
      // switch power supply for measurement resistors OFF
      digitalWrite(eOutPinMeassure, LOW);

      // sort phase
      tempRoofAdc  = simpleSort(tempRoofArray,  sizeof(tempRoofArray)  / sizeof(tempRoofArray[0]));
      tempWaterAdc = simpleSort(tempWaterArray, sizeof(tempWaterArray) / sizeof(tempWaterArray[0]));
      tempSetAdc   = simpleSort(tempSetArray,   sizeof(tempSetArray)   / sizeof(tempSetArray[0]));
      break;

    case eStateConversion:
    {
        tempRoof  = tempConversion(tempRoofAdc,  adcRoofTempValues);
        tempWater = tempConversion(tempWaterAdc, adcWaterTempValues);
        tempSet   = tempConversion(tempSetAdc,   potiTempValues);
    }

    case eStateAction:
    {
      static bool toggleLed = false;

      portStates = 0;           // re-init ports

      // action phases
      if (tempWater < tempSet) {
        // water too cold
        portStates |= eLedHeat;
        if ((!pump && (tempRoof > tempWater + eTempHysteresis)) || (pump && (tempRoof > tempWater))) {
          // either pump already running and roof warmer than water OR pump not running and roof warmer than water + hysteresis
          pump     = true;
        }
        else {
          pump     = false;
          portStates |= eLedMatch;
        }
      }
      else
      if (tempWater > tempSet) {
        // water too warm
        portStates |= eLedCool;
        if ((!pump && (tempRoof < tempWater - eTempHysteresis)) || (pump && (tempRoof < tempWater))) {
          pump     = true;
        }
        else {
          pump     = false;
          portStates |= eLedMatch;
        }
      }
      else {
        // water matches
        pump = false;
        portStates |= eLedMatch;
      }

      // danger of frost?
      if (tempRoof < eTempFrost) {
        portStates |= eLedFrost;
      }

      // toggle LED currently ON?
      if (toggleLed) {
        portStates |= eLedOnBoard;
      }
      toggleLed = !toggleLed;

      // pump ON?
      if (pump) {
        portStates |= eOutPump;
      }


      // set/clear output ports dependent on set bits in ports mask
      for (uint8_t index = 0; index < 16; index++) {
        if (portStates & (1 << index)) {
          digitalWrite(pins[index], pinsActive[index]);        // set pin to "ON", real state depends on output type, low or high active!
        }
        else {
          digitalWrite(pins[index], !pinsActive[index]);       // set pin to "OFF", real state depends on output type, low or high active!
        }
      }
      // @todo falls lastError != 0 dann statt der LEDs den Error Code blinken!

      break;
    }

    case eStateOneWire:
      // get 1-wire temperature
      if (ds18B20Found) {
        enum {
          eStartOneWireMeasurement    = 0x44,
          eStartOneWireScratchPadRead = 0xBE,
        };

        ds1820.reset();
        ds1820.skip();                                        // don't select specific sensor since there is only one supported!
        ds1820.write(eStartOneWireMeasurement, 0);            // start measurement, parasitic supply OFF
        delay(1000);                                          // wait 1 second
        ds1820.reset();
        ds1820.skip();                                        // don't select specific sensor since there is only one supported!

        // read scratch pad
        ds1820.write(eStartOneWireScratchPadRead);
        for (uint8_t loop = 0; loop < 9; loop++) {
          ds18B20Data[loop] = ds1820.read();
        }

        // calculate real temperature from DS18B20
        uint16_t tempRaw = (ds18B20Data[1] << 8) | ds18B20Data[0];
        if (tempRaw & 0x8000) {
          // we don't need negative values so clear them to ZERO for simplier calculation
          tempRaw = 0;
        }

        // we calculate temp from DS18B20 in 1/100°C so we don't need any floating point values!
        ds18B20Value  = tempRaw / 16 * 100;                     // value from sensor = 16 * temp, so integer temp is value / 16, * 100 to get a fixed point value with two positions after decimal point
        tempRaw       = (tempRaw % 16) * 100 / 16;              // fract part of temp is rest of tem * 100 to get two decimal point positions and / 16 to get real value
        ds18B20Value += tempRaw;                                // concat integer part and fraction part together to get a value that contains [1/100°C] value
      }

      break;

    case eStatePrint:
      // print phases
      Serial.print(F("Roof: "));
      Serial.print(tempRoof, DEC);
      Serial.print(F("°C / Water: "));
      Serial.print(tempWater, DEC);
      Serial.print(F("°C / Set: "));
      Serial.print(tempSet, DEC);
      Serial.print(F("°C / Pump: "));
      Serial.print(pump, DEC);
      Serial.print(F(" / LEDs: 0x"));
      Serial.print(portStates, HEX);

      if (ds18B20Found) {
        Serial.print(F(" / DS18B20: "));

        if (OneWire::crc8(ds18B20Data, sizeof(ds18B20Data) - 1) != ds18B20Data[sizeof(ds18B20Data) - 1]) {
          Serial.print(F("CRC invalid, ignore it!"));
        }
        else {
          Serial.print(ds18B20Value / 100, DEC);
          Serial.print(F("."));
          Serial.print(ds18B20Value - (ds18B20Value / 100 * 100), DEC);
          Serial.print(F(" °C/100 / DS18B20 data: "));

          for (uint8_t index = 0; index < sizeof(ds18B20Data); index++) {
            if (ds18B20Data[index] < 0x10) {
              Serial.print(F("0"));
            }
            Serial.print(ds18B20Data[index], HEX);
            Serial.print(F(" "));
          }
        }
      }
      Serial.print(F("\n"));

      Serial.print(F("Roof ADCs: "));
      for (uint8_t loop = 0; loop < sizeof(tempRoofArray)  / sizeof(tempRoofArray[0]); loop++) {
        Serial.print(tempRoofArray[loop], HEX);
        Serial.print(F(" "));
      }
      Serial.print(F("\n"));
      Serial.print(F("Water ADCs: "));
      for (uint8_t loop = 0; loop < sizeof(tempWaterArray)  / sizeof(tempWaterArray[0]); loop++) {
        Serial.print(tempWaterArray[loop], HEX);
        Serial.print(F(" "));
      }
      Serial.print(F("\n"));
      Serial.print(F("Set ADCs: "));
      for (uint8_t loop = 0; loop < sizeof(tempSetArray)  / sizeof(tempRoofArray[0]); loop++) {
        Serial.print(tempSetArray[loop], HEX);
        Serial.print(F(" "));
      }
      Serial.print(F("\n"));

      state = -1;   // set state machine back to first state

      /* @todo analog Ports Kondensator entladen
       *  pinMode(xx, OUTPUT);
       *  digitalWrite(xx, LOW);
       *  delay(1250);
       *  pinMode(xx, INPUT);
       * ggf. auch in der setup() funktion schon mal!!!
       */

      // if there is a DS18B20 then we have already 1 second break... if not we need one here!
      if (!ds18B20Found) {
        delay(1000);
      }

      Serial.print(F("--------------------------------------------\n"));

      break;

    default:
      Serial.print(F("unexpected state: "));
      Serial.print(state, DEC);
      Serial.print(F("\n"));
      assert(false, eError_0002);

      state = -1;   // set state machine back to first state
      break;
  }

  state++;
}
