#include <SPI.h>
#include <TimerOne.h>
#include <EEPROM.h>

#define CS_PIN 9

// ===================================================
// LIMITS
// ===================================================
const unsigned int MAX_STEPS = 300;
const unsigned int MIN_STEPS = 4;


// ===================================================
// EEPROM
// ===================================================

const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_AMPL_ADDR  = 4;

const uint8_t EEPROM_MAGIC = 0x5A;


// ===================================================
// USER PARAMETERS
// ===================================================

unsigned int STEPS = 200;

float FREQ_HZ = 200.0;

float DUTY = 0.0;

// Default only used if EEPROM has never been initialized
float AMPL = 0.2;

unsigned long PAUSE_MS = 500;

// Time for emergency return:
// current DAC A -> 0
unsigned long STOP_MS = 500;


// ===================================================
// RAMP TABLE
// ===================================================

uint16_t spiFrames[MAX_STEPS];

unsigned int upSteps = 0;
unsigned int dnSteps = 0;

unsigned int idx = 0;


// ===================================================
// TIMER FLAGS
// ===================================================

volatile bool updateDAC = false;
volatile bool cycleDone = false;


// ===================================================
// COUNT
// ===================================================

volatile unsigned long pulseCount = 0;

unsigned long targetCount = 0;


// ===================================================
// STATES
// ===================================================

enum RunState
{
    IDLE,
    RUNNING,
    PAUSING,
    STOPPING
};

RunState state = IDLE;


// ===================================================
// PAUSE CONTROL
// ===================================================

unsigned long pauseStart = 0;

// If true, pause is the final pause before stopping
bool stopAfterPause = false;


// ===================================================
// EMERGENCY STOP CONTROL
// ===================================================

unsigned long stopStart = 0;

uint16_t stopInitialA = 0;


// ===================================================
// CURRENT DAC VALUES
//
// RAW 12-bit values only.
//
// 0    = 0 V
// 4095 = full scale
// ===================================================

uint16_t currentDAC_A = 0;

uint16_t currentDAC_B = 0;


// ===================================================
// SERIAL BUFFER
// ===================================================

const byte CMD_BUFFER_SIZE = 64;

char cmdBuffer[CMD_BUFFER_SIZE];

byte cmdIndex = 0;


// ===================================================
// LOAD AMPLITUDE FROM EEPROM
// ===================================================

void loadAmplitudeFromEEPROM()
{
    uint8_t magic;

    EEPROM.get(
        EEPROM_MAGIC_ADDR,
        magic
    );

    if (magic == EEPROM_MAGIC)
    {
        float storedAMPL;

        EEPROM.get(
            EEPROM_AMPL_ADDR,
            storedAMPL
        );

        if (
            storedAMPL >= 0.0 &&
            storedAMPL <= 1.0
        )
        {
            AMPL = storedAMPL;
        }
    }
    else
    {
        // First use
        AMPL = 0.2;

        EEPROM.put(
            EEPROM_MAGIC_ADDR,
            EEPROM_MAGIC
        );

        EEPROM.put(
            EEPROM_AMPL_ADDR,
            AMPL
        );
    }
}


// ===================================================
// SAVE AMPLITUDE TO EEPROM
//
// EEPROM is only written when AMPL changes.
// ===================================================

void saveAmplitudeToEEPROM()
{
    EEPROM.put(
        EEPROM_AMPL_ADDR,
        AMPL
    );
}


// ===================================================
// CALCULATE DAC B
//
// DAC B = HALF OF DAC A PEAK
//
// Example:
//
// AMPL = 0.2
//
// DAC A peak = 4095 × 0.2
//             = 819
//
// DAC B = 819 / 2
//       ≈ 409
//
// For a 5 V DAC:
//
// DAC A peak ≈ 1.0 V
// DAC B      ≈ 0.5 V
// ===================================================

uint16_t calculateBias()
{
    unsigned int peakA =
        (unsigned int)(
            4095.0 * AMPL
        );

    return peakA / 2;
}


// ===================================================
// BUILD DAC-A RAMP TABLE
//
// DAC B is NOT modified here.
// ===================================================

void buildTable()
{
    if (STEPS < MIN_STEPS)
        STEPS = MIN_STEPS;

    if (STEPS > MAX_STEPS)
        STEPS = MAX_STEPS;


    if (DUTY < 0.0)
        DUTY = 0.0;

    if (DUTY > 1.0)
        DUTY = 1.0;


    if (AMPL < 0.0)
        AMPL = 0.0;

    if (AMPL > 1.0)
        AMPL = 1.0;


    // =================================================
    // NUMBER OF RISING POINTS
    // =================================================

    upSteps =
        (unsigned int)(
            DUTY * STEPS
        );


    if (upSteps < 2)
        upSteps = 2;


    if (upSteps > STEPS - 2)
        upSteps = STEPS - 2;


    dnSteps =
        STEPS - upSteps;


    // =================================================
    // BUILD DAC A
    // =================================================

    for (
        unsigned int i = 0;
        i < STEPS;
        i++
    )
    {
        unsigned long v;


        // Rising
        if (i < upSteps)
        {
            v =
                (4095UL * i) /
                (upSteps - 1);
        }

        // Falling
        else
        {
            v =
                (4095UL *
                 (STEPS - 1 - i)) /
                (dnSteps - 1);
        }


        // Apply amplitude
        v =
            (unsigned long)(
                v * AMPL
            );


        if (v > 4095)
            v = 4095;


        spiFrames[i] =
            (uint16_t)(
                v & 0x0FFF
            );
    }


    idx = 0;
}


// ===================================================
// WRITE DAC A ONLY
// ===================================================

inline void writeDAC_A(
    uint16_t value
)
{
    value &= 0x0FFF;


    uint16_t command =
        0x3000 | value;


    digitalWrite(
        CS_PIN,
        LOW
    );


    SPI.transfer(
        highByte(command)
    );

    SPI.transfer(
        lowByte(command)
    );


    digitalWrite(
        CS_PIN,
        HIGH
    );


    currentDAC_A = value;
}


// ===================================================
// WRITE DAC B ONLY
// ===================================================

inline void writeDAC_B(
    uint16_t value
)
{
    value &= 0x0FFF;


    uint16_t command =
        0xB000 | value;


    digitalWrite(
        CS_PIN,
        LOW
    );


    SPI.transfer(
        highByte(command)
    );

    SPI.transfer(
        lowByte(command)
    );


    digitalWrite(
        CS_PIN,
        HIGH
    );


    currentDAC_B = value;
}


// ===================================================
// WRITE BOTH DACs
//
// Used only during initial hardware setup.
// ===================================================

inline void writeBoth(
    uint16_t valueA,
    uint16_t valueB
)
{
    writeDAC_A(valueA);

    writeDAC_B(valueB);
}


// ===================================================
// TIMER ISR
// ===================================================

void timerISR()
{
    updateDAC = true;
}


// ===================================================
// START TIMER
// ===================================================

void applyTimer()
{
    if (FREQ_HZ <= 0)
        FREQ_HZ = 1;


    unsigned long period_us =
        (unsigned long)(
            1000000.0 /
            (FREQ_HZ * STEPS)
        );


    if (period_us < 10)
        period_us = 10;


    Timer1.initialize(
        period_us
    );


    Timer1.attachInterrupt(
        timerISR
    );
}


// ===================================================
// NORMAL STOP
//
// The ramp has already reached its natural endpoint:
//
// DAC A = 0
//
// DAC B remains at Apeak/2.
//
// NO DAC WRITE.
// ===================================================

void normalStop()
{
    Timer1.detachInterrupt();


    noInterrupts();

    updateDAC = false;
    cycleDone = false;

    interrupts();


    state = IDLE;

    stopAfterPause = false;


    Serial.println(
        "STOPPED"
    );
}


// ===================================================
// BEGIN EMERGENCY STOP
//
// Ctrl+C:
//
// Current A
//     ↓
// smoothly A → 0
//
// B DOES NOT CHANGE.
// ===================================================

void beginEmergencyStop()
{
    if (state == IDLE)
        return;


    Timer1.detachInterrupt();


    noInterrupts();

    updateDAC = false;
    cycleDone = false;

    interrupts();


    stopInitialA =
        currentDAC_A;


    stopStart =
        millis();


    state =
        STOPPING;


    Serial.println(
        "EMERGENCY STOPPING"
    );
}


// ===================================================
// UPDATE EMERGENCY STOP
//
// DAC A smoothly goes:
//
// current A → 0
//
// DAC B stays exactly where it is.
// ===================================================

void updateEmergencyStop()
{
    unsigned long elapsed =
        millis() - stopStart;


    // =================================================
    // FINISHED
    // =================================================

    if (
        STOP_MS == 0 ||
        elapsed >= STOP_MS
    )
    {
        // Final state:
        //
        // A = 0
        // B = Apeak / 2
        //
        // B is NOT rewritten.

        writeDAC_A(0);


        state = IDLE;


        Serial.println(
            "EMERGENCY STOPPED"
        );


        return;
    }


    // =================================================
    // CALCULATE REMAINING FRACTION
    // =================================================

    float fraction =
        (float)elapsed /
        (float)STOP_MS;


    float remaining =
        1.0 - fraction;


    uint16_t newA =
        (uint16_t)(
            stopInitialA *
            remaining
        );


    // =================================================
    // ONLY A CHANGES
    // =================================================

    writeDAC_A(
        newA
    );


    delayMicroseconds(1000);
}


// ===================================================
// START RAMP
//
// IMPORTANT:
//
// NO DAC WRITE HERE.
//
// DAC A should already be:
//
// A = 0
//
// DAC B should already be:
//
// B = Apeak/2
//
// Therefore START itself cannot create
// an unwanted DAC-B pulse.
// ===================================================

void startRamp()
{
    if (state != IDLE)
    {
        Serial.println(
            "ERROR NOT IDLE"
        );

        return;
    }


    buildTable();


    idx = 0;

    pulseCount = 0;

    updateDAC = false;

    cycleDone = false;

    stopAfterPause = false;


    // =================================================
    // DO NOT WRITE DAC A
    //
    // DO NOT WRITE DAC B
    //
    // The DACs are already at the correct starting
    // condition.
    // =================================================


    state = RUNNING;


    applyTimer();


    Serial.println(
        "STARTED"
    );
}


// ===================================================
// PROCESS COMMAND
// ===================================================

void processCommand(
    char *cmd
)
{
    // =================================================
    // START
    // =================================================

    if (
        strcmp(
            cmd,
            "START"
        ) == 0
    )
    {
        startRamp();

        return;
    }


    // =================================================
    // STOP
    // =================================================

    if (
        strcmp(
            cmd,
            "STOP"
        ) == 0
    )
    {
        normalStop();

        return;
    }


    // =================================================
    // SET COMMAND
    // =================================================

    if (
        strncmp(
            cmd,
            "SET ",
            4
        ) == 0
    )
    {
        char *space =
            strchr(
                cmd + 4,
                ' '
            );


        if (space == NULL)
            return;


        *space = '\0';


        char *key =
            cmd + 4;


        char *valueString =
            space + 1;


        float val =
            atof(
                valueString
            );


        // =================================================
        // STEPS
        // =================================================

        if (
            strcmp(
                key,
                "STEPS"
            ) == 0
        )
        {
            STEPS =
                (unsigned int)val;


            if (STEPS < MIN_STEPS)
                STEPS = MIN_STEPS;


            if (STEPS > MAX_STEPS)
                STEPS = MAX_STEPS;


            buildTable();


            Serial.print(
                "STEPS "
            );

            Serial.println(
                STEPS
            );
        }


        // =================================================
        // FREQUENCY
        // =================================================

        else if (
            strcmp(
                key,
                "FREQ"
            ) == 0
        )
        {
            FREQ_HZ = val;


            if (FREQ_HZ < 0.1)
                FREQ_HZ = 0.1;


            Serial.print(
                "FREQ "
            );

            Serial.println(
                FREQ_HZ
            );
        }


        // =================================================
        // DUTY
        // =================================================

        else if (
            strcmp(
                key,
                "DUTY"
            ) == 0
        )
        {
            DUTY = val;


            if (DUTY < 0.0)
                DUTY = 0.0;


            if (DUTY > 1.0)
                DUTY = 1.0;


            buildTable();


            Serial.print(
                "DUTY "
            );

            Serial.println(
                DUTY
            );
        }


        // =================================================
        // AMPLITUDE
        // =================================================
        //
        // VERY IMPORTANT:
        //
        // Only allowed while IDLE.
        //
        // B is changed directly to:
        //
        // B = Apeak / 2
        //
        // There is NO 2.5 V intermediate command.
        // =================================================

        else if (
            strcmp(
                key,
                "AMPL"
            ) == 0
        )
        {
            if (state != IDLE)
            {
                Serial.println(
                    "ERROR AMPLITUDE CHANGE ONLY WHEN IDLE"
                );

                return;
            }


            if (val < 0.0)
                val = 0.0;


            if (val > 1.0)
                val = 1.0;


            AMPL = val;


            // Build new A table
            buildTable();


            // Calculate NEW B directly
            uint16_t newBias =
                calculateBias();


            // =================================================
            // CHANGE B DIRECTLY
            //
            // Example:
            //
            // AMPL 0.2
            //
            // B = 0.5 V
            //
            // There is no 2.5 V step generated by software.
            // =================================================

            writeDAC_B(
                newBias
            );


            // Save for future Arduino resets
            saveAmplitudeToEEPROM();


            Serial.print(
                "AMPL "
            );

            Serial.println(
                AMPL
            );


            Serial.print(
                "DACB_CODE "
            );

            Serial.println(
                newBias
            );
        }


        // =================================================
        // PAUSE
        // =================================================

        else if (
            strcmp(
                key,
                "PAUSE"
            ) == 0
        )
        {
            PAUSE_MS =
                (unsigned long)val;


            Serial.print(
                "PAUSE "
            );

            Serial.println(
                PAUSE_MS
            );
        }


        // =================================================
        // STOP TIME
        // =================================================

        else if (
            strcmp(
                key,
                "STOP_MS"
            ) == 0
        )
        {
            STOP_MS =
                (unsigned long)val;


            Serial.print(
                "STOP_MS "
            );

            Serial.println(
                STOP_MS
            );
        }


        // =================================================
        // COUNT
        // =================================================

        else if (
            strcmp(
                key,
                "COUNT"
            ) == 0
        )
        {
            targetCount =
                (unsigned long)val;


            Serial.print(
                "COUNT TARGET "
            );

            Serial.println(
                targetCount
            );
        }
    }
}


// ===================================================
// SERIAL HANDLER
// ===================================================

void handleSerial()
{
    while (
        Serial.available()
    )
    {
        char c =
            Serial.read();


        // =================================================
        // EMERGENCY STOP
        //
        // Python sends:
        //
        // !
        //
        // No newline required.
        // =================================================

        if (c == '!')
        {
            cmdIndex = 0;


            while (
                Serial.available()
            )
            {
                Serial.read();
            }


            beginEmergencyStop();


            return;
        }


        // =================================================
        // NORMAL COMMAND END
        // =================================================

        if (
            c == '\n' ||
            c == '\r'
        )
        {
            if (
                cmdIndex > 0
            )
            {
                cmdBuffer[
                    cmdIndex
                ] = '\0';


                processCommand(
                    cmdBuffer
                );


                cmdIndex = 0;
            }


            continue;
        }


        // =================================================
        // STORE CHARACTER
        // =================================================

        if (
            cmdIndex <
            CMD_BUFFER_SIZE - 1
        )
        {
            cmdBuffer[
                cmdIndex++
            ] = c;
        }
        else
        {
            cmdIndex = 0;


            Serial.println(
                "ERROR COMMAND TOO LONG"
            );
        }
    }
}


// ===================================================
// SETUP
// ===================================================

void setup()
{
    Serial.begin(
        115200
    );


    Serial.setTimeout(
        10
    );


    pinMode(
        CS_PIN,
        OUTPUT
    );


    digitalWrite(
        CS_PIN,
        HIGH
    );


    SPI.begin();


    SPI.beginTransaction(
        SPISettings(
            8000000,
            MSBFIRST,
            SPI_MODE0
        )
    );


    // =================================================
    // LOAD LAST AMPLITUDE FIRST
    //
    // This is important.
    //
    // Arduino does NOT start with AMPL = 0.97 anymore.
    //
    // It remembers the last amplitude.
    // =================================================

    loadAmplitudeFromEEPROM();


    // Build ramp using remembered AMPL
    buildTable();


    // Calculate correct B
    currentDAC_B =
        calculateBias();


    currentDAC_A = 0;


    // =================================================
    // INITIAL HARDWARE STATE
    //
    // A = 0
    // B = Apeak/2
    //
    // No intermediate 2.5 V software command.
    // =================================================

    writeBoth(
        currentDAC_A,
        currentDAC_B
    );


    state = IDLE;


    Serial.println(
        "READY"
    );


    Serial.print(
        "AMPL "
    );

    Serial.println(
        AMPL
    );


    Serial.print(
        "DACB_CODE "
    );

    Serial.println(
        currentDAC_B
    );
}


// ===================================================
// MAIN LOOP
// ===================================================

void loop()
{
    // =================================================
    // SERIAL FIRST
    // =================================================

    handleSerial();


    // =================================================
    // EMERGENCY STOPPING
    // =================================================

    if (
        state == STOPPING
    )
    {
        updateEmergencyStop();

        return;
    }


    // =================================================
    // IDLE
    // =================================================

    if (
        state == IDLE
    )
    {
        return;
    }


    // =================================================
    // RUNNING
    // =================================================

    if (
        updateDAC &&
        state == RUNNING
    )
    {
        updateDAC = false;


        // =================================================
        // ONLY DAC A CHANGES
        //
        // B stays constant.
        // =================================================

        writeDAC_A(
            spiFrames[idx]
        );


        idx++;


        // =================================================
        // COMPLETE RAMP
        // =================================================

        if (
            idx >= STEPS
        )
        {
            idx = 0;


            pulseCount++;


            Serial.print(
                "COUNT "
            );

            Serial.println(
                pulseCount
            );


            cycleDone = true;
        }
    }


    // =================================================
    // CYCLE FINISHED
    // =================================================

    if (
        cycleDone &&
        state == RUNNING
    )
    {
        cycleDone = false;


        // =================================================
        // TARGET REACHED
        //
        // The ramp has naturally finished.
        //
        // A is already at 0.
        //
        // B remains Apeak/2.
        //
        // We now perform the requested pause.
        // =================================================

        if (
            targetCount > 0 &&
            pulseCount >= targetCount
        )
        {
            stopAfterPause = true;


            pauseStart =
                millis();


            state =
                PAUSING;


            Serial.print(
                "DONE "
            );

            Serial.println(
                pulseCount
            );


            return;
        }


        // =================================================
        // NORMAL PAUSE
        // =================================================

        stopAfterPause = false;


        pauseStart =
            millis();


        state =
            PAUSING;
    }


    // =================================================
    // PAUSE
    // =================================================

    if (
        state == PAUSING
    )
    {
        // =================================================
        // NO DAC WRITES HERE
        //
        // A remains at 0.
        // B remains Apeak/2.
        // =================================================

        if (
            millis() -
            pauseStart >=
            PAUSE_MS
        )
        {
            // Final target reached
            if (stopAfterPause)
            {
                normalStop();

                return;
            }


            // Otherwise start next cycle
            updateDAC = false;

            idx = 0;

            state =
                RUNNING;


            applyTimer();
        }
    }
}