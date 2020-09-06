/***********************************
 * @TODO
 *   - Werte der pump-/nicht-pump-Zeiten noch im EEPROM speichern
 *   - DCF77 support, da sonst Uhrzeit und Datum nicht klar sind
 *   - Pump-Zeiten werden auch generell noch nicht berücksichtigt, aktuell läuft alles nur über die Temperatur
 *
 *
 *
 *
 *
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
 *
 * D-Sub:
 *      Buchse      Stecker
 *      intern      extern
 *
 *     1 (RT) -(GND)- (WS) 1
 *     2 (SW) -(+5V)- (BL) 2
 *     6 (WS) -(D+)-- (BN) 6
 *     7 (GN) -(D-)-- (GN) 7
 *
 */

// all following DEBUG defines are only active if DEBUG is also defined, so with DEBUG all debugging stuff can be enabled and disabled!
#define DEBUG                   // define for DEBUG enabled
#define SENSOR_FOUND            // define to simulate sensors even if none is connected
#define TEMPERATURE_READ        // define to simulate sensor temperature read even if no sensor has been connected
#define TEMPERATURE_SET         // define to simulate potentiometer read even if no potentiometer has been connected
#define DEBUG_PINS              // manipulate measured water and roof temp by setting a debug pin to LOW


#include <OneWire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>

// I2C address for OLED display: 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C


// Arduino nano pinning
#define D0   0              // RX
#define D1   1              // TX
#define D2   2              // 1-wire bus (onewire)
#define D3   3              // pump ON + heater valve closed (closed means water is pumped to the roof)
#define D4   4              // set to low increases roof  temp by 5°C (for debugging)
#define D5   5              // set to low increases water temp by 5°C (for debugging)
#define D6   6              // debug output pin (toggled n times for simple debugging)
#define D7   7              // LED red   "heizen"
#define D8   8              // LED red   "Solltemperatur erreicht"
#define D9   9              // LED green "kühlen"
#define D10 10              // LED red   "Frostgefahr"
#define D11 11              // Button
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
#define A4  4               // I2C SDA
#define A5  5               // I2C SCL
#define A6  6               //
#define A7  7               //


enum {
    eMenuEntries     = 5,
    eExitMenuIndex   = eMenuEntries,            // last menu entry is a general menu entry (Exit)!
    eMenuEntriesSize = 21,
};
typedef void (*menuFunction_mt)(void);
typedef struct
{
    char menuEntry[eMenuEntries][eMenuEntriesSize];
    menuFunction_mt menuFunction[eMenuEntries];
} menu_mt;


#if defined DEBUG && defined TEMPERATURE_READ
uint16_t fakeTempRoof  = 20 * 100;      // 20°C fake temp in case of debugging
uint16_t fakeTempWater = 20 * 100;      // 20°C fake temp in case of debugging
#endif
#if defined DEBUG && defined TEMPERATURE_SET
uint16_t fakeTempSet   = 20 * 100;      // 20°C fake temp in case of debugging
#endif
#if defined DEBUG && (defined TEMPERATURE_READ || defined TEMPERATURE_SET)
enum { eFakeTempStep = 50 };       // increment/decrement fake temps by 0.5°C
#endif



/**
 * error buffer variables
 */
uint16_t errorBuffer[10];           // to store any error for blinken lights if any happened
uint16_t errorBufferIndex = 0;      // index where next error can be stored


/**
 * print enable/disable variables
 */
bool printAdcConversion = false;    // set to TRUE to print ADC conversion information
bool printVariables     = false;    // set to TRUE to print variables every state machine turn


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
    eTempOnHysteresis    = 500,  // start pump when roof temp >  water temp +/- ON hysteresis (unit: 1/100°C)
    eTempOffHysteresis   = 300,  // stop  pump when roof temp <= water temp +/- OFF hysteresis (unit: 1/100°C)
    eTempMatchHysteresis = 50,   // show "match" when water temp within set temp +/- MATCH hysteresis (unit: 1/100°C)
    eTempFrost           = 5,    // show frost when this temp or less has been reached
    eTempSamples         = 6,    // amount of temp samples (must be 2^n+2 because of sort and median algorithm!)
};



enum { eResumableErrorThreshold = 5 };
uint8_t resumableError = 0;                 // count errors until threshold reached


/**
 * states for loop state machine
 */
enum {
    eStateInitial        = 0,
    eStateSamplePhase,
    eStateMedian,
    eStateConversion,
    eStateOneWire,
    eStateAction,
    eStateOled,
    eStateButtonPressed,
    eStatePrint,

    eStateInitialError,
    eStateOneWireError,
    eStateHaltError,
};

// @todo Poti in Steuerung nochmal durchmessen ob ADC-Werte noch stimmen (Versorgungsspannung hat sich ja verändert!!!)

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

    eInPinDebugRoof  = D4,      // if set to 1 roof  temperature will be incremented by 5°C for testing
    eInPinDebugWater = D5,      // if set to 1 water temperature will be incremented by 5°C for testing
    eOutPinDebug     = D6,      // toggled several times at different code positions for simple debugging

    eLedPinHeat      = D7,      // LED pin for heating
    eLedPinMatch     = D8,      // LED pin if set temperature reached
    eLedPinCool      = D9,      // LED pin for cooling
    eLedPinFrost     = D10,     // LED pin at risk of frost

    eLedPinError     = D10,     // blink LED pin in case of error

    eInPinButton     = D11,     // input for menu operations

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
//                        enum Bits:  eLedHeat     eLedMatch     eLedCool     eLedFrost     eLedOnBoard     eOutPump
const uint8_t pins[]               = {eLedPinHeat, eLedPinMatch, eLedPinCool, eLedPinFrost, eLedPinOnBoard, eOutPinPump};       // order should be identical with previous enum!!!
const uint8_t pinsActive[]         = {LOW,         LOW,          LOW,         LOW,          HIGH,           HIGH};              // values to set refering output to ON
const char    pinsOnNames[][6][10] = {{"heat",     "match",      "cool",      "frost",      "onboard",      "pump"},
                                      {"HEAT",     "MATCH",      "COOL",      "FROST",      "ONBOARD",      "PUMP"}};
const uint8_t pinsInitial[]        = {LOW,         LOW,          LOW,         LOW,          HIGH,           LOW};               // values to initialize refering outputs


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
 */
const uint16_t potiTempValues[] = {
    0x13A,                    // 10°C (this value will be taken if ADC value is lower!)
    0x172,                    // 15°C
    0x1B9,                    // 20°C
    0x1FE,                    // 25°C
    0x243,                    // 30°C
    0x290,                    // 35°C
    0x2E0,                    // 40°C
    0x2E0 + (0x2E0 - 0x297),  // 45°C (calculated!)
    0
};


// OLED display object
SSD1306AsciiAvrI2c oled;


enum {
    eButtonNotPressed      = 0,
    eButtonShortPressed    = 1,
    eButtonLongPressed     = 2,
};
enum {
    eMenuEntryRedraw    = true,
    eMenuEntryRefresh   = false,
};
static uint8_t lastButtonPressed = eButtonNotPressed;
/**
 * does button have been pressed?
 * 0 = button not pressed
 * 1 = short pressed (release has been seen in time)
 * 2 = long pressed (no release has been seen in time)
 */
uint8_t buttonPressed()
{
    enum {
        eShortButtonTime = 100,
        eLongButtonTime  = 1000,
    };
    if (digitalRead(eInPinButton) == LOW)
    {
        unsigned long pressedTimeStamp = millis();
        while (((millis() - pressedTimeStamp) < eLongButtonTime) && (digitalRead(eInPinButton) == LOW));

        unsigned long pressedTime = millis() - pressedTimeStamp;

        if (pressedTime < eShortButtonTime)
        {
            // button not pressed or filtered by software debouncing
            lastButtonPressed = eButtonNotPressed;
        }
        else
        if (pressedTime < eLongButtonTime)
        {
                // after a long press (with button not released we can detect a ghostly short press when this function has been called twice and user pressed long + a bit longer, so in that case button has been released now and not already pressed again!)
                if (lastButtonPressed == eButtonLongPressed)
                {
                    lastButtonPressed = eButtonNotPressed;
                }
                else
                {
                    // button has been pressed short since it was released in previous call!
                    lastButtonPressed = eButtonShortPressed;
                }
        }
        else
        {
            // button was pressed long (or is still pressed since last time)
            lastButtonPressed = eButtonLongPressed;
        }
    }
    else
    {
        // button currently not pressed
        lastButtonPressed = eButtonNotPressed;
    }

    return lastButtonPressed;
}


/***********************************************************************************/
/**
 *
 */
void printMenuExtra()
{
    oled.clear();
    oled.set1X();
    oled.println("untermenue");
    unsigned long currentDeltaMillis = millis();

    while(millis() - currentDeltaMillis < 1000);
}


/**
 * Expects a value between 0 and 99 and will print it (for hours usually it should be between 0 and 23, for minutes it should be between 0 and 59)
 */
void oledPrintTime(uint8_t timeValue, bool inverted)
{
    if (inverted)
    {
        oled.setInvertMode(true);
    }

    oled.print(timeValue / 10 ? (char)(timeValue / 10 + '0') : (char)'0');
    oled.print(timeValue % 10 ? (char)(timeValue % 10 + '0') : (char)'0');

    oled.setInvertMode(false);
}


/**
 *
 */
void enterTimeRange(char * name, uint8_t times[4])
{
    oled.clear();
    oled.set2X();

    uint8_t selectIndex = 0;


    while (selectIndex < 4)
    {
        oled.home();

        oled.println(name);
        oled.print("from:");
        oledPrintTime(times[0], selectIndex == 0);
        oled.print(":");
        oledPrintTime(times[1], selectIndex == 1);
        oled.println();

        oled.print("to:  ");
        oledPrintTime(times[2], selectIndex == 2);
        oled.print(":");
        oledPrintTime(times[3], selectIndex == 3);

        // wait for user input
        while (!buttonPressed());

        if (lastButtonPressed == eButtonShortPressed)
        {
            // increment selected time element
            times[selectIndex]++;

            if ((selectIndex) % 2)
            {
                // minute selected
                times[selectIndex] = times[selectIndex] % 60;
            }
            else
            {
                // hour selected
                times[selectIndex] = times[selectIndex] % 24;
            }
        }
        else
        {
            // select next time element
            selectIndex++;
        }
    }
}


char pumpString[] = "pump on\n";
uint8_t pumpOnTimes[4] = {
    10,     // hour of start time
     0,     // minute of start time
    16,     // hour of end time
     0      // minute of end time
};
char nightString[] = "pump off\n";
uint8_t pumpOffTimes[4] = {
    18,     // hour of start time
     0,     // minute of start time
     9,     // hour of end time
     0      // minute of end time
};
uint8_t pumpOnTimesWinter[4] = {
    11,     // hour of start time
     0,     // minute of start time
    14,     // hour of end time
     0      // minute of end time
};
uint8_t pumpOffTimesWinter[4] = {
    14,     // hour of start time
     0,     // minute of start time
    11,     // hour of end time
     0      // minute of end time
};
/**
 *
 */
void enterPumpTime()
{
    enterTimeRange(pumpString, pumpOnTimes);
}
/**
 *
 */
void enterNightTime()
{
    enterTimeRange(nightString, pumpOffTimes);
}
/**
 *
 */
void enterPumpTimeWinter()
{
    enterTimeRange(pumpString, pumpOnTimesWinter);
}
/**
 *
 */
void enterNightTimeWinter()
{
    enterTimeRange(nightString, pumpOffTimesWinter);
}


menu_mt startMenu = {
    {
     // "123456789012345678901", no '\n'!
        "pump ON time",
        "pump OFF time",
        "winter pump ON",
        "winter night OFF",
    },
    {
        enterPumpTime,
        enterNightTime,
        enterPumpTimeWinter,
        enterNightTimeWinter,
    }
};


/**
 * print a given menu structure adding an extra menu entry "Exit" at the end and adding a ">" character in front of selected index.
 * In case user just switches through the menu parameter "redraw" should be false to prevent flickering, when a new menu has to be printed "redraw" should be set to true so the screen is cleared at the beginning
 */
void printMenu(menu_mt * menu, uint8_t menuIndex, bool redraw)
{
    oled.set1X();

    if (redraw == eMenuEntryRedraw)
    {
        oled.clear();
    }
    else
    {
        oled.home();
    }

//    oled.println("123456789A123456789B123456789C123456789D123456789E123456789F");

    uint8_t index;
    for (index = 0; index < eMenuEntries; index++)
    {
        if (menu->menuFunction[index] != NULL)
        {
            oled.print(index == menuIndex ? ">" : " ");
            oled.println(menu->menuEntry[index]);
        }
        else
        {
            oled.println("");
        }
    }

    oled.println("");
    oled.print(menuIndex == eExitMenuIndex ? ">" : " ");
    oled.println("Exit");
}


/**
 * Handles a given menu strucutre, elements can be selected (auto-repeat is supported by pressing the button for a long time), menu elements can be executed
 */
void showMenu(menu_mt * menu)
{
    uint8_t menuIndex = 0;
    bool    leaveMenu = false;

    printMenu(menu, menuIndex, eMenuEntryRedraw);

    while (buttonPressed());        // wait for user releasing the button

    while (!leaveMenu)
    {
        if (buttonPressed())
        {
            // short or long pressed?
            if (lastButtonPressed == eButtonShortPressed)
            {
                if (menuIndex == eExitMenuIndex)
                {
                    leaveMenu = true;     // leave current menu
                }
                else
                {
                    // short pressed, so enter menu
                    menu->menuFunction[menuIndex]();

                    // print menu since either index changed or called menu function has been left again
                    printMenu(menu, menuIndex, eMenuEntryRedraw);
                }
            }
            else
            if (lastButtonPressed == eButtonLongPressed)
            {
                // long pressed, so step to next menu entry
                do
                {
                    menuIndex = (menuIndex + 1) % (eMenuEntries + 1);           // "eMenuEntries + 1" since there is a general menu entry "Exit" existing in each menu and just returns!
                } while ((menu->menuFunction[menuIndex] == NULL) && (menuIndex < eMenuEntries));

                // print menu since either index changed or called menu function has been left again
                printMenu(menu, menuIndex, eMenuEntryRefresh);
            }
        }
    }
}



/**
 * Returns current error state
 *
 * @return      TRUE in case error already happened, FALSE otherwise
 */
static inline bool isError(void) {
    return (errorBufferIndex != 0);
}


/**
 * Clear error buffer, e.g. in case error disappeared
 * It's helpful in the case that it was possible to "re-setup" after an error happened, e.g. during initialization phase by doing it again
 */
static inline void clearErrorBuffer(void) {
    errorBufferIndex = 0;
}


/**
 * Blinks given error code to frost and onboard LED
 *
 * @error   error value to blink
 */
void blinkError(uint16_t error) {
    for (uint8_t blinkenLights = 0; blinkenLights < error; blinkenLights++) {
        // LEDs on
        digitalWrite(eLedPinError,   pinsActive[eLedFrostIndex]);       // for user
        digitalWrite(eLedPinOnBoard, pinsActive[eLedOnBoardIndex]);     // for debugging
        delay(500);

        // LEDs off
        digitalWrite(eLedPinError,   !pinsActive[eLedFrostIndex]);      // for user
        digitalWrite(eLedPinOnBoard, !pinsActive[eLedOnBoardIndex]);    // for debugging
        delay(500);
    }
    delay(2000);
}


/**
 * Toggles debug pin for n times and waits 50ms afterwards, so debugging with analyzer will become possible
 */
#if defined DEBUG
void debugToggle(uint8_t amount) {
    while (amount--) {
        digitalWrite(eOutPinDebug, HIGH);
        digitalWrite(eOutPinDebug, LOW);
    }
    delay(1);
}
#else
#   define debugToggle(x)           /* remove that code if DEBUG is not activated! */
#endif


/**
 * assert function to check something and set error if necessary
 *
 * @successful      true in case test was OK, false if test failed
 * @error           uniq error number to be stored
 */
bool assert(bool successful, eError_mt error) {
    if (!successful) {

        digitalWrite(eLedPinOnBoard, !pinsActive[eLedOnBoardIndex]);        // for debugging
        delay(5);
        for (uint16_t toggle = 0; toggle < error; toggle++) {
            digitalWrite(eLedPinOnBoard, pinsActive[eLedOnBoardIndex]);     // for debugging
            digitalWrite(eLedPinOnBoard, !pinsActive[eLedOnBoardIndex]);    // for debugging
        }
        delay(1);

        // store error in case error buffer is not yet full
        if (errorBufferIndex < sizeof(errorBuffer)/sizeof(errorBuffer[0])) {

            // don't set same error twice directly one after another
            if ((errorBufferIndex == 0) || (errorBuffer[errorBufferIndex - 1] != error)) {
                errorBuffer[errorBufferIndex] = error;
                errorBufferIndex++;
            }
        }
    }

    return successful;
}


/**
 * sets all outputs to "OFF" state
 */
void shutOff(void) {
    // clear all output ports in error case
    for (uint8_t index = 0; index < sizeof(pins); index++) {
        digitalWrite(pins[index], !pinsActive[index]);       // set pin to "OFF", real state depends on output type, low or high active!
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

    if (printAdcConversion) {
        Serial.print(F("Calculate: degreeDelta: "));
        Serial.print(degreeDelta, DEC);
    }

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

        if (printAdcConversion) {
            Serial.print(F(" / adcValue: "));
            Serial.print(adcValue, HEX);
            Serial.print(F(" / adcDelta: "));
            Serial.print(adcDelta, HEX);
        }
    }

    if (printAdcConversion) {
        Serial.print(F(" / result degree: "));
        Serial.print(result, DEC);
        Serial.print(F("\n"));
    }

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

    if (printAdcConversion) {
        Serial.print(F("Convert: "));
        Serial.print(PTC ? F("PTC") : F("NTC"));
        Serial.print(F(" / adc: "));
        Serial.print(adcValue, HEX);
    }

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

    if (printAdcConversion) {
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
    }

    // table start temperature is 10°C, table entry step is 5°C wise
    return tempCalculation(adcValue, 10 + (5 * lowIndex), 10 + (5 * highIndex), value, nextValue);
}


/**
 * 1-wire commands
 */
enum {
    eStartOneWireMeasurement    = 0x44,
    eStartOneWireScratchPadRead = 0xBE,
};


/**
 * initiates 1-wire measurement.
 * after calling this function you should watit ~1 second before reading scratch pad!
 *
 * @sensorAddress   address of DS18B20 to be read
 */
void initiateSensorMeasurement(const uint8_t * sensorAddress) {
    ds1820.reset();
    ds1820.select(sensorAddress);                   // select sensor
    ds1820.write(eStartOneWireMeasurement, 0);      // initiate measurement
}


/**
 * reads 1-wire scratch pad
 *
 * @sensorAddress   address of DS18B20 to be read
 */
uint16_t readSensorScratchpad(const uint8_t * sensorAddress, uint8_t * data) {
    uint16_t result;

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
 * Reads temperature from DS18B20 sensor, stores data in given data array (e.g. for print) and returns received temperature
 * In case of error an error will be stored.
 *
 * @sensorAddress   address of DS18B20 to be read
 * @data            to store raw received sensor data
 *
 * @return          temperature in 1/100°C
 */
uint16_t getSensorTemperature(const uint8_t * sensorAddress, uint8_t * data) {
    // initiate measurement
    initiateSensorMeasurement(sensorAddress);
    delay(1000);                                    // wait 1 second (>= 750ms!)

    return readSensorScratchpad(sensorAddress, data);
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


/**
 * To print error buffer content
 */
void printErrorBuffer(void) {
    for (uint8_t index = 0; index < errorBufferIndex; index++) {
        Serial.print(F("Error "));
        Serial.print(index, DEC);
        Serial.print((": "));
        Serial.print(errorBuffer[index], HEX);
        Serial.println("");
    }
    Serial.println("");
}


/**
 * To print error buffer content
 */
void printHelp(void) {
    Serial.print(F("h: print this help screen\n"));
    Serial.print(F("a: print ADC conversion ON/OFF\n"));
    Serial.print(F("p: print variables ON/OFF\n"));
#if defined DEBUG && defined TEMPERATURE_READ
    Serial.print(F("w: decrement faked water temperature by 0.5°C\n"));
    Serial.print(F("W: increment faked water temperature by 0.5°C\n"));
    Serial.print(F("r: decrement faked roof  temperature by 0.5°C\n"));
    Serial.print(F("R: increment faked roof  temperature by 0.5°C\n"));
#endif
#if defined DEBUG && defined TEMPERATURE_SET
    Serial.print(F("s: decrement faked set temperature by 0.5°C\n"));
    Serial.print(F("S: increment faked set temperature by 0.5°C\n"));
#endif
}



/**
 * print a single line on OLED display show temperature information, e.g. "O: 14.0 C"
 */
void printTemperatureToOled(char meaning, uint16_t temperature)
{
    uint16_t tempInt   = temperature / 100;             // part in front of decimal point
    uint16_t tempFract = temperature % 100 / 10;        // only first digit after decimal point will be shown!

                                        // "12345678901"
    oled.print(meaning);                // "X"
    oled.print(": ");                   // "X: "

    if (tempInt < 10)
    {
        oled.print(" ");                // "X:  "
    }

    oled.print(tempInt);                // "X:  4"
    oled.print(".");                    // "X:  4."
    oled.print(tempFract);              // "X:  4.0"

    oled.println(" C");                 // "X:  4.0 C"
}


/**
 * setup function
 */
void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);           //  setup serial

    // initialize debug pins
    pinMode(eInPinDebugRoof,  INPUT_PULLUP);
    pinMode(eInPinDebugWater, INPUT_PULLUP);
    pinMode(eOutPinDebug,     OUTPUT);

    // initialize outputs
    pinMode(eOutPinPump,      OUTPUT);
    pinMode(eLedPinHeat,      OUTPUT);

    // initialize LEDs ports
    pinMode(eLedPinMatch,     OUTPUT);
    pinMode(eLedPinCool,      OUTPUT);
    pinMode(eLedPinFrost,     OUTPUT);
    pinMode(eLedPinOnBoard,   OUTPUT);

    // initialize input ports
    pinMode(eInPinButton,     INPUT_PULLUP);

    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Adafruit5x7);
    oled.clear();
    oled.set2X();

//#define PRINT_CHAR_TABLE
#if defined PRINT_CHAR_TABLE
    for (uint16_t charIndex = 32; charIndex < 256; charIndex++)
    {
        oled.print((char)charIndex);

        if (!((charIndex - 31) % 10))
        {
            oled.println("");
        }
        if (!((charIndex - 31) % (10 * 4)))
        {
            delay(5000);
            oled.clear();
        }
    }
    
    delay(5000);
    oled.clear();
#endif

    oled.println("INIT...");

    // initialize output values
    for (uint8_t index = 0; index < sizeof(pins); index++) {
        digitalWrite(pins[index], pinsInitial[index]);       // set pin to initial value
    }

    printHelp();
}


bool oledPrint = false;                  // OLED output will be printed if set to true
unsigned long oledPrintMillis = 0;       // timestamp when OLED output has been set to true (to switch display of after some seconds again)
static inline void switchOledOn()
{
    oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
    oledPrint = true;
    oledPrintMillis = millis();
}


/**
 * loop function (worker thread)
 */
void loop() {
    static int16_t state = eStateInitial;           // current state machine state

    static bool menuMode = false;                   // user entered menu mode

    static bool oledInitiallyShown = false;

    if (!oledInitiallyShown)
    {
        oledInitiallyShown = true;
        switchOledOn();
    }

    if (!menuMode)
    {
        debugToggle(1);

        if (state <= eStateHaltError) {
            debugToggle(state + 1);
        }
        else {
            debugToggle(eStateHaltError + 10);
        }

        switch (state) {
            case eStateInitial:
            {
                clearErrorBuffer();           // clear errors in case we are here not for the first time... in that case try to initialize again and again and again...

                // search for a 1-wire sensors (first two supported all others will just be shown), 1st one is water temp, 2nd one is roof temp
                uint8_t address[8];
                ds1820.reset_search();
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

    #if defined DEBUG && defined SENSOR_FOUND
                clearErrorBuffer();
    #endif

                // no error during sensor search?
                if (!isError()) {
                    for (uint8_t index = 0; index < eSensorIndexMax; index++) {
                        sensorTemp[index] = getSensorTemperature(sensorAddresses[index], sensorData[index]);
                    }
                }

    #if defined DEBUG && defined TEMPERATURE_READ
                clearErrorBuffer();
                sensorTemp[eSensorIndexRoof]  = fakeTempRoof;
                sensorTemp[eSensorIndexWater] = fakeTempWater;
    #endif

                // no error during temperature read?
                if (!isError()) {
                    state = eStateSamplePhase;
                    resumableError = 0;         // clear resumable errors if any occurred
                }
                else {
                    // in case error happend up to here switch to error handling state
                    state = eStateInitialError;
                }

                break;
            }

            case eStateSamplePhase:
            {
                static uint8_t sampleCounter = 0;

                tempSetArray[sampleCounter] = analogRead(eTempPinSet);

                sampleCounter = (sampleCounter + 1) % eTempSamples;     // execute this state eTempSamples times!

                // when previous module division sets sampleCounter to zero again one whole set of measurements has been finished
                if (sampleCounter == 0) {
                    state = eStateMedian;
                }

                break;
            }

            case eStateMedian:      // sort phase for measured set temperatures
                tempSetAdc = simpleSort(tempSetArray, sizeof(tempSetArray) / sizeof(tempSetArray[0]));
                state = eStateConversion;

                break;

            case eStateConversion:  // convert set temperature
                tempSet = tempConversion(tempSetAdc, potiTempValues) * 100;
                state = eStateOneWire;

    #if defined DEBUG && defined TEMPERATURE_SET
                clearErrorBuffer();
                tempSet = fakeTempSet;
    #endif

                break;

            case eStateOneWire:
            {
                static uint8_t oneWireCounter = 0;

                enum {
                    eInitiateSensorMeasurement = 0,
                    eReadRoofSensor            = 10,
                    eReadWaterSensor,         // 11
                    eMeasurementLoopTurns,    // 12
                };

                clearErrorBuffer();           // clear errors in case we are here not for the first time... in that case try to read temperature again and again and again...


                /* this state contains different jobs:
                 * 1st      run = initiate measurement at both 1-wire sensors
                 * 2nd..9th run = do nothing
                 * 10th     run = read temperature from first  sensor
                 * 11th     run = read temperature from second sensor
                 *
                 * the delay(100) will ensure that independent from the time all the other states need one complete state machine run will take at least 100ms... so all together we have >= 1 second for the sensors to do their temperature measurements
                 */

                if (oneWireCounter == eInitiateSensorMeasurement) {
                    // initiate temperature measurement on both sensors
                    initiateSensorMeasurement(sensorAddresses[0]);
                    initiateSensorMeasurement(sensorAddresses[1]);
                }
                else
                if (oneWireCounter == eReadRoofSensor) {
                    // read temperatur from first sensor
                    sensorTemp[eSensorIndexRoof] = readSensorScratchpad(sensorAddresses[eSensorIndexRoof], sensorData[eSensorIndexRoof]);

    #if defined DEBUG && defined TEMPERATURE_READ
                    clearErrorBuffer();
                    sensorTemp[eSensorIndexRoof] = fakeTempRoof;
    #endif

    #if defined DEBUG && defined DEBUG_PINS
                    if (digitalRead(eInPinDebugRoof) == LOW) {
                        // D5 (= roof temp + 5°C)
                        sensorTemp[eSensorIndexRoof] += 100;
                    }
    #endif
                }
                else
                if (oneWireCounter == eReadWaterSensor) {
                    // read temperatur from second sensor
                    sensorTemp[eSensorIndexWater] = readSensorScratchpad(sensorAddresses[eSensorIndexWater], sensorData[eSensorIndexWater]);

    #if defined DEBUG && defined TEMPERATURE_READ
                    clearErrorBuffer();
                    sensorTemp[eSensorIndexWater] = fakeTempWater;
    #endif

    #if defined DEBUG && defined DEBUG_PINS
                    if (digitalRead(eInPinDebugWater) == LOW) {
                        // D4 (= water temp + 5°C)
                        sensorTemp[eSensorIndexWater] += 100;
                    }
    #endif
                }

                // no error so far?
                if (!isError()) {
                    oneWireCounter = (oneWireCounter + 1) % eMeasurementLoopTurns;
                }
                // else "repeat current one wire action"

                delay(100);

                state = eStateAction;

                break;
            }

            case eStateAction:
            {
                /* initially we start with "MATCH" per default
                 *  - start HEATing when water is colder than or equal to set temp - 0.5°C
                 *  - stop  HEATing when water is warmer than or equal to set temp + 0.5°C
                 *  - start COOLing when water is warmer than             set temp + 1.0°C
                 *  - stop  COOLing when water is colder than or equal to set temp
                 *  - pump  will be off when water temperature is in: ]set temp - 0.5°C, set temp + 1.0°C]
                 *
                 *  - if HEAT has been chosen
                 */

                static bool toggleLed = false;

                portStates = 0;           // re-init ports

                // action phases
                if (((heatCool == eHeatStateMatch) && (sensorTemp[eSensorIndexWater] <= tempSet - eTempMatchHysteresis)) || ((heatCool == eHeatStateHeat) && (sensorTemp[eSensorIndexWater] < tempSet + eTempMatchHysteresis))) {
                    // water too cold
                    heatCool = eHeatStateHeat;

                    /* To get some hysteresis pump will be ON when:
                     *  - if pump is OFF while roof temp > water temp + hysteresis
                     *  - if pump is ON  while roof temp > water temp
                     */

                    portStates |= eLedHeat;
                    if ((!pump && (sensorTemp[eSensorIndexRoof] > sensorTemp[eSensorIndexWater] + eTempOnHysteresis)) || (pump && (sensorTemp[eSensorIndexRoof] > sensorTemp[eSensorIndexWater] + eTempOffHysteresis))) {
                        // either pump already running and roof warmer than water OR pump not running and roof warmer than water + hysteresis
                        pump     = true;
                    }
                    else {
                        pump     = false;
                        portStates |= eLedMatch;
                    }
                }
                else if (((heatCool == eHeatStateMatch) && (sensorTemp[eSensorIndexWater] > tempSet + 2 * eTempMatchHysteresis)) || ((heatCool == eHeatStateCool) && (sensorTemp[eSensorIndexWater] > tempSet))) {
                    // water too warm
                    heatCool = eHeatStateCool;

                    /* To get some hysteresis pump will be ON when:
                     *  - if pump is OFF while roof temp < water temp - hysteresis
                     *  - if pump is ON  while roof temp < water temp
                     */

                    portStates |= eLedCool;
                    if ((!pump && (sensorTemp[eSensorIndexRoof] < sensorTemp[eSensorIndexWater] - eTempOnHysteresis)) || (pump && (sensorTemp[eSensorIndexRoof] < sensorTemp[eSensorIndexWater] + eTempOffHysteresis))) {
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
                if (toggleLed) {
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

                state = eStateOled;

                break;
            }

            case eStateOled:
                {
                    static uint16_t oldRoofTemp  = 0;
                    static uint16_t oldWaterTemp = 0;

                    static unsigned long lastDeltaMillis = 0;            // it can happen that shown value is wrong for the first time but all following loop runs will be fine!
                    static unsigned long delta = 0;
                    unsigned long currentDeltaMillis = millis();

                    if (oledPrint)
                    {
                        if (millis() - oledPrintMillis > 5000)
                        {
                            oledPrint = false;
                        }

                        if ((oldRoofTemp != sensorTemp[eSensorIndexRoof]) || (oldWaterTemp != sensorTemp[eSensorIndexWater]) || ((currentDeltaMillis - lastDeltaMillis) != delta))
                        {
                            delta = currentDeltaMillis - lastDeltaMillis;
                            oldRoofTemp  = sensorTemp[eSensorIndexRoof];
                            oldWaterTemp = sensorTemp[eSensorIndexWater];

                            oled.home();        // no clear, just go home but ensure that whole display will be filled to overwrite old stuff... so flickering will be suppressed!
                            oled.set2X();

                            // line 1
                            printTemperatureToOled('O', sensorTemp[eSensorIndexRoof]);

                            // line 2
                            printTemperatureToOled('U', sensorTemp[eSensorIndexWater]);

                            // line 3
                            printTemperatureToOled('S', tempSet);

                            oled.set1X();
                            // line 4
                            oled.println("                      ");
//                            oled.println(delta);

                            // line 5
                            if (heatCool == eHeatStateMatch)
                            {
                                oled.print("match");
                            }
                            else
                            if (heatCool == eHeatStateCool)
                            {
                                oled.print("cool");
                            }
                            else
                            if (heatCool == eHeatStateHeat)
                            {
                                oled.print("heat");
                            }
                            oled.print(" | ");
                            oled.print(pump ? "ON" : "OFF");
                            oled.print((portStates & eLedFrost) ? "| freez" : "       ");
                            oled.println("          ");
                        }

                    }
                    else
                    {
                        // oled.clear();
                        oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
                    }

                    lastDeltaMillis = millis();

                    state = eStateButtonPressed;

                    break;
                }

            case eStateButtonPressed:
                {
                    if (buttonPressed())
                    {
                        if (lastButtonPressed == eButtonLongPressed)
                        {
                            // show menu
                            menuMode = true;
                        }
                        else
                        {
                            // show default display information
                            switchOledOn();
                        }
                    }

                    state = eStatePrint;

                    break;
                }

            case eStatePrint:
                {
                    // if user connected terminal to read data he/she can also print a key to see something!
                    int userInput = Serial.read();

                    switch((char)userInput)
                    {
                        case 'h':
                            printHelp();
                            break;

                        case 'a':
                            printAdcConversion = !printAdcConversion;
                            break;

                        case 'p':
                            printVariables = !printVariables;
                            break;

    #if defined DEBUG && defined TEMPERATURE_READ
                        case 'w':
                            if (fakeTempWater > 0) {
                                fakeTempWater -= eFakeTempStep;
                            }
                            break;

                        case 'W':
                            if (fakeTempWater < 8000) {
                                fakeTempWater += eFakeTempStep;
                            }
                            break;

                        case 'r':
                            if (fakeTempRoof > 0) {
                                fakeTempRoof -= eFakeTempStep;
                            }
                            break;

                        case 'R':
                            if (fakeTempRoof < 8000) {
                                fakeTempRoof += eFakeTempStep;
                            }
                            break;
    #endif

    #if defined DEBUG && defined TEMPERATURE_SET
                        case 's':
                            if (fakeTempSet > 0) {
                                fakeTempSet -= eFakeTempStep;
                            }
                            break;

                        case 'S':
                            if (fakeTempSet < 8000) {
                                fakeTempSet += eFakeTempStep;
                            }
                            break;
    #endif
                    }

                    if (printVariables) {
                        // print phases
                        printInformation();
                    }

                    /* @todo analog Ports Kondensator entladen
                    *  pinMode(xx, OUTPUT);
                    *  digitalWrite(xx, LOW);
                    *  delay(1250);
                    *  pinMode(xx, INPUT);
                    * ggf. auch in der setup() funktion schon mal!!!
                    */

                    state = eStateSamplePhase;   // set state machine back to first state

                    break;
                }

            case eStateInitialError:
            case eStateOneWireError:
            {
                blinkError(errorBuffer[0]);

                printErrorBuffer();
                printInformation();

                // increment resumableError until threshold reached
                if (resumableError < eResumableErrorThreshold) {
                    resumableError++;

                    if (state == eStateInitialError) {
                        state = eStateInitial;              // stay in eStateInitial until the two expected sensors have been found
                    }
                    else
                    if (state == eStateOneWireError) {
                        state = eStateOneWire;              // stay in eStateOneWire until reading scratch pad with temperature was successful
                    }
                }
                else {
                    // error threshold reached, so switch over to blocking state
                    state = eStateHaltError;
                }

                break;
            }

            default:
                assert(false, eError_0002);
                // fallthrough
            case eStateHaltError:
                // never leave this state again!
                shutOff();

                blinkError(errorBuffer[0]);

                printErrorBuffer();
                printInformation();

                delay(2000);

                break;
        }
    }
    else
    {
        oled.ssd1306WriteCmd(SSD1306_DISPLAYON);

        // enter menu
        showMenu(&startMenu);

        // leave menu
        menuMode = false;

        // ensure default menu will be printed some secons after leaving menu...
        switchOledOn();
    }
}
