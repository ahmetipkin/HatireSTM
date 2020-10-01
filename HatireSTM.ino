#include "Arduino.h"

#include <EEPROM.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

typedef struct {
    int16_t Begin;   // 2  Debut
    uint16_t Cpt;      // 2  Compteur trame or Code info or error
    float gyro[3];   // 12 [Y, P, R]    gyro
    float acc[3];    // 12 [x, y, z]    Acc
    int16_t End;      // 2  Fin
} _hatire;

typedef struct {
    int16_t Begin;   // 2  Debut
    uint16_t Code;     // 2  Code info
    char Msg[24];   // 24 Message
    int16_t End;      // 2  Fin
} _msginfo;

typedef struct {
    sensors_event_t orientation;

} _eprom_save;

constexpr float Rad2Deg = (180 / M_PI);

char Version[] = "HAT V 1.10";
const byte INTERRUPT_PINNUMBER = 19;                    // where the INT is attached.

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

_eprom_save calibrationOffsets;

// MPU control/status vars
bool bnoReady = false;
bool active = false;

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

_hatire hatire;
_msginfo msginfo;

bool AskCalibrate = false;  // set true when calibrating is ask
int CptCal = 0;
const int NbCal = 5;

constexpr int CAL_BUTTON = 32;

//The setup function is called once at startup of the sketch
void setup() {
// Add your initialization code here

    pinMode(CAL_BUTTON, INPUT_PULLDOWN);

    delay(1000);
    Serial.begin();
    PrintCodeSerial(2000, Version, true);

    hatire.Begin = 0xAAAA;
    hatire.Cpt = 0;
    hatire.End = 0x5555;

    msginfo.Begin = 0xAAAA;
    msginfo.Code = 0;
    msginfo.End = 0x5555;

    PrintCodeSerial(3001, "Initializing I2C", true);

    if (!bno.begin()) {
        PrintCodeSerial(9007, "BNO055 ERROR BEGIN", true);
    } else {
        PrintCodeSerial(3003, "BNO055 CONNECTION OK", true);
        bnoReady = true;
    }

    PrintCodeSerial(3002, "Initializing EEPROM", true);
    EEPROM.begin();

    while (Serial.available() && Serial.read())
        ; // empty buffer

    PrintCodeSerial(3012, "READY", true);

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calibrate() {
    CptCal = 0;
    razoffset();
    AskCalibrate = true;
}

void print_status() {
    char msg[24] = { 0 };
    uint8_t sys, gyro, accel, magnet;
    bno.getCalibration(&sys, &gyro, &accel, &magnet);

    sprintf(msg, "S:%hhu, G:%hhu, A:%hhu, M:%hhu", sys, gyro, accel, magnet);
    PrintCodeSerial(3007, msg, true);

}
// The loop function is called in an endless loop
void loop() {
//Add your repeated code here
    if (Serial.available() > 0)
        serialEvent();

    if (digitalRead(CAL_BUTTON) == 1) {
        calibrate();
    }

    if (active && bnoReady) {
        sensors_event_t measurement, acceleration;

        bno.getEvent(&measurement, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&acceleration, Adafruit_BNO055::VECTOR_LINEARACCEL);

        if (AskCalibrate) {
            if (CptCal >= NbCal) {
                CptCal = 0;
                calibrationOffsets.orientation.orientation.x = calibrationOffsets.orientation.orientation.x / NbCal;
                calibrationOffsets.orientation.orientation.y = calibrationOffsets.orientation.orientation.y / NbCal;
                calibrationOffsets.orientation.orientation.z = calibrationOffsets.orientation.orientation.z / NbCal;
                AskCalibrate = false;
                SaveParams();
            } else {
                calibrationOffsets.orientation.orientation.x += measurement.orientation.x;
                calibrationOffsets.orientation.orientation.y += measurement.orientation.y;
                calibrationOffsets.orientation.orientation.z += measurement.orientation.z;

                CptCal++;
            }

            hatire.gyro[0] = 0;
            hatire.gyro[1] = 0;
            hatire.gyro[2] = 0;
            hatire.acc[0] = 0;
            hatire.acc[1] = 0;
            hatire.acc[2] = 0;
        } else {
            float rawX = measurement.orientation.x - calibrationOffsets.orientation.orientation.x;

            // order: z , y , x
            hatire.gyro[0] = measurement.orientation.z - calibrationOffsets.orientation.orientation.z;
            hatire.gyro[1] = measurement.orientation.y - calibrationOffsets.orientation.orientation.y;
            hatire.gyro[2] =  rawX > 180.0 ? rawX - 360.0 : rawX; // map from 0-360 => -180 | 0 | +180  interval.

            hatire.acc[0] = acceleration.acceleration.x;
            hatire.acc[1] = acceleration.acceleration.y;
            hatire.acc[2] = acceleration.acceleration.z;

        }

        Serial.write((byte*) &hatire, 30);

        hatire.Cpt++;
        if (hatire.Cpt > 999) {
            hatire.Cpt = 0;
        }
    }

    delay(1);
}

// ================================================================
// ===                    Serial Command                        ===
// ================================================================
void serialEvent() {
    char commande = (char) Serial.read();
    switch (commande) {
        case 'S':
            PrintCodeSerial(5001, "HAT START", true);
            if (bnoReady == true) {
                hatire.Cpt = 0;
                active = true;
            } else {
                PrintCodeSerial(9011, "Error BNO doesn't work", true);
            }
            break;

        case 's':
            PrintCodeSerial(5002, "HAT STOP", true);
            active = false;
            break;

        case 'R':
            PrintCodeSerial(5003, "HAT RESET", true);

            bnoReady = false;
            setup();

            break;

        case 'Z':
            razoffset();

            break;

        case 'T':
            print_status();
            break;

        case 'C':
            calibrate();
            break;

        case 'V':
            PrintCodeSerial(2000, Version, true);
            break;

        case 'I': {
            Serial.println();
            Serial.print("Version : \t");
            Serial.println(Version);
            Serial.println("Gyroscopes offsets");
            Serial.print("0 : ");
            Serial.println(calibrationOffsets.orientation.orientation.z);
            Serial.print("1 : ");
            Serial.println(calibrationOffsets.orientation.orientation.y);
            Serial.print("2 : ");
            Serial.println(calibrationOffsets.orientation.orientation.x);

            PrintCodeSerial(3005, "Calibration Sequence", true);

            while ((!bno.isFullyCalibrated()) && (digitalRead(CAL_BUTTON) != 1)) {

                PrintCodeSerial(3006, "Calibrating...", false);

                print_status();

                delay(250);
            }
            ReadParams();

            PrintCodeSerial(3007, "Calibration Complete", false);
        }
            break;

        default:
            break;
    }
}

// ================================================================
// ===               PRINT SERIAL FORMATTE                      ===
// ================================================================
void PrintCodeSerial(uint16_t code, char Msg[24], bool EOL) {
    msginfo.Code = code;
    memset(msginfo.Msg, 0x00, 24);
    strcpy(msginfo.Msg, Msg);
    if (EOL)
        msginfo.Msg[23] = 0x0A;
    // Send HATIRE message to  PC
    Serial.write((byte*) &msginfo, 30);
}

// ================================================================
// ===                    RAZ OFFSET                            ===
// ================================================================
void razoffset() {
    calibrationOffsets.orientation.orientation.x = 0;
    calibrationOffsets.orientation.orientation.y = 0;
    calibrationOffsets.orientation.orientation.z = 0;
}

// ================================================================
// ===                    SAVE PARAMS                           ===
// ================================================================
void SaveParams() {
    EEPROM.put(0, calibrationOffsets);
}

// ================================================================
// ===                    READ PARAMS                           ===
// ================================================================
void ReadParams() {
    EEPROM.get(0, calibrationOffsets);
}
