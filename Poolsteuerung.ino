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
 *        roof/water                     temp set                                                                                                                                       used cable is a 3x1,5 JFLEX ([1], [2] and [GG] are the single wires!)
 *
 *            5V                            5V                                    1k5                                                 VCC         ~24V         L(230V)                    VCC [1]
 *            +                              +                                    ___    LED                                           +            +             +                        +
 *            |                              |                            VCC +--|___|---|>|-----+ uC                                  |            |             |                        |
 *            |                              |                                            \\                                     .-----o            |             |                        o-----.
 *           .-.                            .-.                                                                                  |     |            |             |                        |     |
 *           | | 6k8                    1k5 | |                                                                                  -    _|_           o             o                       .-.    |
 *           | |                            | |                                                                                  ^   |_/_|- - - - -/- - - - - - -/                    4k7 | |    |    DS18B20
 *           '-'                            '-'                                                                                  |     |          /             /                         | |    |    ______
 *            |                              |                                                                                   '-----o        o/  o         o/  o                       '-'    '---|3 VDD |
 *   uC +-----o---------.                    |                                                                      10k                |        |   |         |   |                        |         |      |
 *            |         |                   .-.                                                                     ___              |/    open |   | close       |             uC [GG] +--o---------|2 DQ  |
 *           .-.        |               10k | |<--o----o----+ uC                                            uC +---|___|----o-----o--|          |   |             |                                  |      |
 *           | | 5k    --- 10n              | |   |    |                                                                    |     |  |>         |   |             |                     .------------|1_GND_|
 *           | | NTC   ---                  '-'   |    |                                                                    o    .-.   |         \ /              |                     |
 *           '-'        |                    |    |    |                                                                   /     | |2k |         .-.              |                    ===
 *            |         |                    o----'    |                                                      manual |-^--/      | |   |        /   \             |                    GND [2]
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
 *     3 2 2 2 2 2 2 2 2 2 2 1 1 1 1
 *     0 9 8 7 6 5 4 3 2 1 0 9 8 7 6
 *    +-----------------------------+
 *    |o o o o o o o o o o o o o o o|                                       ISP
 *    |                [TX]     ISP |                                      +---+
 *    +-+              [RX]     o o |                                  GND |o o| -RESET
 *    | | USB   [RESET]         o o |             [PB2/PCINT2/MOSI]   MOSI |o o| SCK  [PB1/PCINT1/SCK]
 *    +-+              [PWR]    o x |                                   5V |o x| MISO [PB3/PCINT3/MISO]
 *    |                [D13]        |                                      +---+
 *    |o o o o o o o o o o o o o o o|
 *    +-----------------------------+
 *     1 2 3 4 5 6 7 8 9 1 1 1 1 1 1
 *                       0 1 2 3 4 5
 *
 *
 *     1: D13 (on board LED)       7: A3       13: RESET          19: GND        25: D7
 *     2: 3V3                      8: A4       14: GND            20: D2         26: D8
 *     3: Ana.REF                  9: A5       15: VIN (<=8V)     21: D3         27: D9
 *     4: A0                      10: A6       16: TXD            22: D4         28: D10
 *     5: A1                      11: A7       17: RXD            23: D5         29: D11
 *     6: A2                      12: 5V       18: RESET          24: D6         30: D12
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
#define D4   4              //
#define D5   5              // set to low increases water temp by 5°C (for debugging)
#define D6   6              // set to low increases roof  temp by 5°C (for debugging)
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


uint16_t errorBuffer[10];           // to store any error for blinken lights if any happened
uint16_t errorBufferIndex = 0;      // index where next error can be stored


/**
 * Data from DS18B20 have different length, we have to now that length to calculate proper CRCs
 */
enum {
    eDs18B20LengthAddress     = 8,
    eDs18B20LengthTemperature = 9,
};


/**
 * some constant values
 */
enum {
    eTempHysteresis      = 300,  // don't start pump until temp > temp +/- hysteresis has been reached (unit: 1/100°C)
    eTempMatchHysteresis = 50,   // show "match" when water temp within set temp +/- match hysteresis (+/- 0.5°C)
    eTempFrost           = 5,    // show frost when this temp or less has been reached

    eTempSamples         = 6,    // amount of temp samples (must be 2^n+2 because of sort and median algorithm!)
    eStateMedian         = eTempSamples,
    eStateConversion,
    eStateOneWire,
    eStateAction,
    eStatePrint,
};


/**
 * uniq error values
 */
typedef enum {
    eError_none = 0,
    eError_0001 = 0x0001,     // unexpected length for array to be sorted
    eError_0002 = 0x0002,     // unexpected state reached in state machine
    eError_0003 = 0x0003,     // given ADC value is smaller than given lowTempDegree  and sensor is NTC
    eError_0004 = 0x0004,     // given ADC value is larger  than given highTempDegree and sensor is NTC
    eError_0005 = 0x0005,     // given ADC value is larger  than given lowTempDegree  and sensor is PTC
    eError_0006 = 0x0006,     // given ADC value is smaller than given highTempDegree and sensor is PTC
    eError_0007 = 0x0007,     // CRC error while initially scan for ds18b20
    eError_0008 = 0x0008,     // unknown ds18b20 found
    eError_0009 = 0x0009,     // roof sensor missed
    eError_000A = 0x000A,     // water sensor missed
    eError_000B = 0x000B,     // CRC error while reading temperature from sensor
    eError_000C = 0x000C,     // sensor data in initialization phase was completely zero (DS18B20 CRC is worthless if e.g. pull-up fails, then all data is 0x00 and CRC is also 0x00)
    eError_000D = 0x000D,     // sensor data while reading temperature from sensor was completely zero (DS18B20 CRC is worthless if e.g. pull-up fails, then all data is 0x00 and CRC is also 0x00)
} eError_mt;


/**
 * IOs used for ADC and digital IOs
 */
enum {
    eTempPinSet      = A0,      // pin for set temp

    eOutPinOneWire   = D2,      // one wire pin (to setup temperature values)
    eOutPinPump      = D3,      // pin for pump relais

    eInPinDebugWater = D4,
    eInPinDebugRoof  = D5,

    eLedPinHeat      = D7,      // LED pin for heating
    eLedPinMatch     = D8,      // LED pin if set temperature reached
    eLedPinCool      = D9,      // LED pin for cooling
    eLedPinFrost     = D10,     // LED pin at risk of frost

    eLedPinError     = D10,     // blink LED pin in case of error

    eLedPinOnBoard   = D13,     // Arduino nano onboard LED
};


// latch to pin matrix
enum {
    eLedOff     = 0,        // OFF usually not used

    eLedHeatIndex    = 0,
    eLedMatchIndex   = 1,
    eLedCoolIndex    = 2,
    eLedFrostIndex   = 3,
    eLedOnBoardIndex = 4,
    eOutPumpIndex    = 5,

    eLedHeat    = 1 << eLedHeatIndex   ,   // water needs to be heated
    eLedMatch   = 1 << eLedMatchIndex  ,   // water temperature matches set temperatur
    eLedCool    = 1 << eLedCoolIndex   ,   // water needs to be cooled down
    eLedFrost   = 1 << eLedFrostIndex  ,   // ON in case temp of roof is less than or equal to eTempFrost since then there is any risk that water freezes!
    eLedOnBoard = 1 << eLedOnBoardIndex,   // on board LED blinks as alive signal
    eOutPump    = 1 << eOutPumpIndex   ,   // ON in case pump is running and heater valve has to be closed (= heating/cooling)
};
//                   enum Bits:  eLedHeat     eLedMatch     eLedCool     eLedFrost     eLedOnBoard     eOutPump
const uint8_t pins[]               = {eLedPinHeat, eLedPinMatch, eLedPinCool, eLedPinFrost, eLedPinOnBoard, eOutPinPump};          // order should be identical with previous enum!!!
const uint8_t pinsActive[]         = {LOW,         LOW,          LOW,         LOW,          HIGH,           HIGH};                 // value to set refering output to ON
const char    pinsOnNames[][6][10] = {{"heat",     "match",      "cool",      "frost",      "onboard",      "pump"},
                                      {"HEAT",     "MATCH",      "COOL",      "FROST",      "ONBOARD",      "PUMP"}};


// indices for following sensorAddresses array
enum {
    eSensorIndexRoof  = 0,
    eSensorIndexWater = 1,
    eSensorIndexMax
};


// expected 1-wire sensors
const uint8_t sensorAddresses[2][8] = {
    { 0x28, 0xB2, 0xB7, 0x26, 0x00, 0x00, 0x80, 0x9C },         // roof  sensor
    { 0x28, 0xFF, 0x21, 0x4F, 0x54, 0x14, 0x00, 0x35 }          // water sensor
};
bool     sensorFound[2] = { false, false };
uint16_t sensorTemp[2];
uint8_t  sensorData[2][12];


// variables for main-loop
uint16_t tempSetArray[eTempSamples];    // samples of set temperature potentiometer
uint16_t tempSetAdc;                    // set temp ADC value
uint16_t tempSet;                       // resulting set temp value

enum {
    eHeatStateCool  = -1,
    eHeatStateMatch =  0,
    eHeatStateHeat  =  1,
};
bool     pump       = false;            // current pump state
uint8_t  portStates = 0;                // current LEDs states
int8_t   heatCool   = eHeatStateMatch;  // current heat/cool state, 0 = match, 1 = heat, -1 = cool

// one wire object
OneWire ds1820(eOutPinOneWire);


/**
 * meassured values for potentiometer
 * (*) = calculated                           (*)                                        (*)    (*)    (*)  zero_terminated
 *                                   0xDF  0x127  0x16F  0x1BB  0x200  0x257  0x2AC  0x301  0x365
 */
const uint16_t potiTempValues[] = {   223,   295,   367,   443,   512,   599,   684,   769,   854,  0 };


/**
 * Returns current error state
 *
 * @return      TRUE in case error already happened, FALSE otherwise
 */
static inline bool isError(void) {
    return (errorBufferIndex != 0);
}


/**
 * assert function to check something and set error if necessary
 *
 * @successful      true in case test was OK, false if test failed
 * @error           uniq error number to be stored
 */
static inline bool assert(bool successful, eError_mt error) {
    if (!successful) {
        Serial.print(F("Error: "));
        Serial.println(error);

        // store error in case error buffer is not yet full
        if (errorBufferIndex < sizeof(errorBuffer)/sizeof(errorBuffer[0])) {
            errorBuffer[errorBufferIndex++] = error;
        }
    }

    return successful;
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
    else if (adcValue == highTempAdc) {
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
 * Reads temperature from DS18B20 sensor, stores data in given data array (e.g. for print) and returns received temperature
 * In case of error an error will be stored.
 *
 * @sensorAddress   address of DS18B20 to be read
 * @data            to store raw received sensor data
 *
 * @return          temperature in 1/100°C
 */
uint16_t getSensorTemperature(const uint8_t * sensorAddress, uint8_t * data) {
    enum {
        eStartOneWireMeasurement    = 0x44,
        eStartOneWireScratchPadRead = 0xBE,
    };

    uint16_t result;

    // initiate measurement
    ds1820.reset();
    ds1820.select(sensorAddress);                   // select sensor
    ds1820.write(eStartOneWireMeasurement, 0);      // initiate measurement
    delay(1000);                                    // wait 1 second (>= 750ms!)

    // read temperature
    ds1820.reset();
    ds1820.select(sensorAddress);                   // select sensor
    ds1820.write(eStartOneWireScratchPadRead);      // read scratch pad

    for (uint8_t loop = 0; loop < eDs18B20LengthTemperature; loop++) {
        data[loop] = ds1820.read();
    }
    uint16_t rawTemp = (data[1] << 8) | data[0];

    assert(OneWire::crc8(data, eDs18B20LengthTemperature - 1) == data[eDs18B20LengthTemperature - 1], eError_000B);

    assert(!memncmpx(data, eDs18B20LengthTemperature, 0x00), eError_000D);

    // we don't need negative values so clear them to 0°C for simplier calculation
    if (rawTemp & 0x8000) {
        rawTemp = 0;
    }

    // we calculate temp from DS18B20 in 1/100°C so we don't need any floating point values!
    result  = rawTemp / 16 * 100;                     // value from sensor = 16 * temp, so integer temp is value / 16, * 100 to get a fixed point value with two positions after decimal point
    rawTemp = (rawTemp % 16) * 100 / 16;              // fract part of temp is rest of tem * 100 to get two decimal point positions and / 16 to get real value
    result += rawTemp;                                // concat integer part and fraction part together to get a value that contains [1/100°C] value

    return result;
}


/**
 * Compares array with given value
 *
 * @return  TRUE in case all elements of array are identical with value, FALSE otherwise
 */
bool memncmpx(uint8_t * data, uint8_t length, uint8_t value) {
    bool equal = true;

    for (uint8_t index = 0; (index < length) && equal; index++) {
        equal &= (data[index] == value);
    }

    return equal;
}



/**
 * Since sensor temp contains temperature in 1/100°C degree it's necessary to print the first digits followed by a decimal point followed by the remaining two digits...
 */
void printSensorTemp(uint16_t sensorTemp) {
    Serial.print(sensorTemp / 100, DEC);
    Serial.print(".");
    Serial.print(sensorTemp % 100, DEC);
}


/**
 * prints DS18B20 address from given address array
 */
void printHexBytes(const uint8_t * data, uint8_t length) {
    for (uint8_t index = 0; index < length; index++) {
        if (data[index] <= 0xF) {
            Serial.print("0");
        }
        Serial.print(data[index],HEX);
        Serial.print(" ");
    }
}


/**
 * To print all collected information (usually last phase in state machine but also useful in error case)
 */
void printInformation(void) {
    Serial.print(F("Roof: "));
    printSensorTemp(sensorTemp[eSensorIndexRoof]);
    Serial.print(F("°C / Water: "));
    printSensorTemp(sensorTemp[eSensorIndexWater]);
    Serial.print(F("°C / Set: "));
    printSensorTemp(tempSet);
    Serial.print(F("°C / Pump: "));
    Serial.print(pump ? F("ON") : F("OFF"));
    Serial.print(F(" / heatCool: "));
    Serial.print(heatCool == eHeatStateMatch ? F("MATCH") : heatCool == eHeatStateHeat ? F("HEAT") : F("COOL"));
    Serial.print(F(" / LEDs: 0b0"));
    Serial.print(portStates, BIN);
    Serial.print(F(" [ "));
    for (uint8_t index = 0; index < sizeof(pins); index++) {
        if (portStates & (1 << index)) {
            Serial.print(pinsOnNames[1][index]);
        }
        else {
            Serial.print(pinsOnNames[0][index]);
        }
        Serial.print(" ");
    }
    Serial.print(F("]\n"));

    Serial.print(F("Set ADCs: "));
    for (uint8_t loop = 0; loop < sizeof(tempSetArray) / sizeof(tempSetArray[0]); loop++) {
        Serial.print(tempSetArray[loop], HEX);
        Serial.print(F(" "));
    }
    Serial.print(F("\n"));

    Serial.print(F("DS18B20 Roof Adr:  "));
    printHexBytes(sensorAddresses[eSensorIndexRoof], eDs18B20LengthAddress);
    Serial.print(F(" / Data: "));
    printHexBytes(sensorData[eSensorIndexRoof], eDs18B20LengthTemperature);
    Serial.println("");

    Serial.print(F("DS18B20 Water Adr: "));
    printHexBytes(sensorAddresses[eSensorIndexWater], eDs18B20LengthAddress);
    Serial.print(F(" / Data: "));
    printHexBytes(sensorData[eSensorIndexWater], eDs18B20LengthTemperature);
    Serial.println("");

    Serial.print(F("--------------------------------------------\n"));
}


void printErrorBuffer(void) {
    for (uint8_t index = 0; index < errorBufferIndex; index++) {
        Serial.print(F("Error: "));
        Serial.print(errorBuffer[index], HEX);
        Serial.println("");
    }
    Serial.println("");
}


/**
 * setup function
 */
void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);           //  setup serial

    // initialize debug inputs
    pinMode(eInPinDebugWater, INPUT_PULLUP);
    pinMode(eInPinDebugRoof,  INPUT_PULLUP);

    // initialize outputs
    pinMode(eOutPinPump,      OUTPUT);
    pinMode(eLedPinHeat,      OUTPUT);

    // initialize LEDs ports
    pinMode(eLedPinMatch,     OUTPUT);
    pinMode(eLedPinCool,      OUTPUT);
    pinMode(eLedPinFrost,     OUTPUT);
    pinMode(eLedPinOnBoard,   OUTPUT);

    // search for a 1-wire sensors (first two supported all others will just be shown), 1st one is water temp, 2nd one is roof temp
    uint8_t address[8];
    ds1820.reset();
    while (ds1820.search(address)) {
        Serial.print(F("found DS18B20: "));
        printHexBytes(address, eDs18B20LengthAddress);

        if (!assert(OneWire::crc8(address, eDs18B20LengthAddress - 1) == address[eDs18B20LengthAddress - 1], eError_0007)) {
            Serial.print(F(" : invalid CRC"));
        }
        else
        if (!assert(!memncmpx(address, eDs18B20LengthAddress, 0x00), eError_000C)) {
            Serial.print(F(" : zero data"));
        }
        else {
            if (strncmp((const char*)address, (const char*)sensorAddresses[eSensorIndexRoof], sizeof(sensorAddresses[eSensorIndexRoof])) == 0) {
                sensorFound[eSensorIndexRoof] = true;
            }
            else
            if (strncmp((const char*)address, (const char*)sensorAddresses[eSensorIndexWater], sizeof(sensorAddresses[eSensorIndexWater])) == 0) {
                sensorFound[eSensorIndexWater] = true;
            }
            else {
                assert(false, eError_0008);
            }
        }
        Serial.println("");
    }

    assert(sensorFound[eSensorIndexRoof],  eError_0009);
    assert(sensorFound[eSensorIndexWater], eError_000A);

    if (!isError()) {
        for (uint8_t index = 0; index < eSensorIndexMax; index++) {
            sensorTemp[index] = getSensorTemperature(sensorAddresses[index], sensorData[index]);
            sensorTemp[index] = getSensorTemperature(sensorAddresses[index], sensorData[index]);
        }
    }
}


/**
 * loop function (worker thread)
 */
void loop() {
    static int16_t  state = 0;                      // current state machine state

    // in case of error don't start anything but blink error code!
    if (isError()) {
        // clear all output ports in error case
        for (uint8_t index = 0; index < sizeof(pins); index++) {
            digitalWrite(pins[index], !pinsActive[index]);       // set pin to "OFF", real state depends on output type, low or high active!
        }

        while(1) {      // endless loop in error case
            for (uint8_t blinkenLights = 0; blinkenLights < errorBuffer[0]; blinkenLights++) {
                digitalWrite(eLedPinError,   pinsActive[eLedFrostIndex]);       // for user
                digitalWrite(eLedPinOnBoard, pinsActive[eLedOnBoardIndex]);     // for debugging
                delay(500);
                digitalWrite(eLedPinError,   !pinsActive[eLedFrostIndex]);      // for user
                digitalWrite(eLedPinOnBoard, !pinsActive[eLedOnBoardIndex]);    // for debugging
                delay(500);
            }
            delay(2000);

            printErrorBuffer();
            printInformation();
        }
    }

    // in case of no error execute state machine
    if (state < eTempSamples) {
        // sample phases
        tempSetArray[state]   = analogRead(eTempPinSet);
    }
    else {
        switch (state) {
            case eStateMedian:      // sort phase for measured set temperatures
                tempSetAdc = simpleSort(tempSetArray, sizeof(tempSetArray) / sizeof(tempSetArray[0]));
                break;

            case eStateConversion:  // convert set temperature
                tempSet = tempConversion(tempSetAdc, potiTempValues) * 100;
                break;

            case eStateOneWire:     // get 1-wire temperatures
                static uint8_t sensorIndex = 0;

                // get temperature from one sensor (otherwise state machine cycle will take too long!)
                sensorTemp[sensorIndex] = getSensorTemperature(sensorAddresses[sensorIndex], sensorData[sensorIndex]);

                if (digitalRead(eInPinDebugWater) == LOW) {
                    if (sensorIndex == eSensorIndexWater) {
                        sensorTemp[eSensorIndexWater] += 500;
                    }
                }
                else
                if (digitalRead(eInPinDebugRoof) == LOW) {
                    if (sensorIndex == eSensorIndexRoof) {
                        sensorTemp[eSensorIndexRoof] += 500;
                    }
                }

                // select next sensor index
                sensorIndex++;
                if (sensorIndex >= eSensorIndexMax) {
                    sensorIndex = 0;
                }
                break;

            case eStateAction:
            {
                /* initially we start with "MATCH" per default
                 *  - if water is colder than set temp - 1.0°C then "HEAT" is selected
                 *  - if "HEAT" was already selected it stays selected as long as water is colder than set temp + 0.5°C so it won't toggle between heating and cooling
                 *  - if "HEAT" was already selected and water reaches set temp + 0.5°C then "MATCH" will be selected
                 *  - if water is warmer than set temp + 1.0°C then "COOL" is selected
                 *  - if "COOL" was already selected it stays selected as long as water is warmer than set temp - 0.5°C so it won't toggle between heating and cooling
                 *  - if "COOL" was already selected and water reaches set temp - 0.5°C then "MATCH" will be selected
                 *  - if "MATCH" is selected it will stay selected as long as water is within [set temp - 1.0°C, set temp + 1.0°C ]
                 */

                static bool toggleLed = false;

                portStates = 0;           // re-init ports

                // action phases
                if (((heatCool == eHeatStateMatch) && (sensorTemp[eSensorIndexWater] < tempSet - 2 * eTempMatchHysteresis)) || ((heatCool == eHeatStateHeat) && (sensorTemp[eSensorIndexWater] < tempSet + eTempMatchHysteresis))) {
                    // water too cold
                    heatCool = eHeatStateHeat;

                    /* To get some hysteresis pump will be ON when:
                     *  - if pump is OFF while roof temp > water temp + hysteresis
                     *  - if pump is ON  while roof temp > water temp
                     */

                    portStates |= eLedHeat;
                    if ((!pump && (sensorTemp[eSensorIndexRoof] > sensorTemp[eSensorIndexWater] + eTempHysteresis)) || (pump && (sensorTemp[eSensorIndexRoof] > sensorTemp[eSensorIndexWater]))) {
                        // either pump already running and roof warmer than water OR pump not running and roof warmer than water + hysteresis
                        pump     = true;
                    }
                    else {
                        pump     = false;
                        portStates |= eLedMatch;
                    }
                }
                else if (((heatCool == eHeatStateMatch) && (sensorTemp[eSensorIndexWater] > tempSet + 2 * eTempMatchHysteresis)) || ((heatCool == eHeatStateCool) && (sensorTemp[eSensorIndexWater] > tempSet - eTempMatchHysteresis))) {
                    // water too warm
                    heatCool = eHeatStateCool;

                    /* To get some hysteresis pump will be ON when:
                     *  - if pump is OFF while roof temp < water temp - hysteresis
                     *  - if pump is ON  while roof temp < water temp
                     */

                    portStates |= eLedCool;
                    if ((!pump && (sensorTemp[eSensorIndexRoof] < sensorTemp[eSensorIndexWater] - eTempHysteresis)) || (pump && (sensorTemp[eSensorIndexRoof] < sensorTemp[eSensorIndexWater]))) {
                        pump     = true;
                    }
                    else {
                        pump     = false;
                        portStates |= eLedMatch;
                    }
                }
                else {
                    // water matches
                    heatCool = eHeatStateMatch;

                    pump = false;
                    portStates |= eLedMatch;
                }

                // danger of frost?
                if (sensorTemp[eSensorIndexRoof] < eTempFrost) {
                    portStates |= eLedFrost;
                }

                // toggle LED currently ON?
                if (toggleLed && !isError()) {
                    portStates |= eLedOnBoard;
                }
                toggleLed = !toggleLed;

                // pump ON?
                if (pump) {
                    portStates |= eOutPump;
                }


                // set/clear output ports dependent on set bits in ports mask
                for (uint8_t index = 0; index < sizeof(pins); index++) {
                    if (portStates & (1 << index)) {
                        digitalWrite(pins[index], pinsActive[index]);        // set pin to "ON", real state depends on output type, low or high active!
                    }
                    else {
                        digitalWrite(pins[index], !pinsActive[index]);       // set pin to "OFF", real state depends on output type, low or high active!
                    }
                }

                break;
            }

        case eStatePrint:
            // print phases
            printInformation();

            state = -1;   // set state machine back to first state

            /* @todo analog Ports Kondensator entladen
             *  pinMode(xx, OUTPUT);
             *  digitalWrite(xx, LOW);
             *  delay(1250);
             *  pinMode(xx, INPUT);
             * ggf. auch in der setup() funktion schon mal!!!
             */

            break;

            default:
                assert(false, eError_0002);

                state = -1;   // set state machine back to first state
                break;
        }
    }
    state++;
}
