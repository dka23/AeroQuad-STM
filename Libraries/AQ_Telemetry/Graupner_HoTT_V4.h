//
//  Graupner_HoTT_V4.h
//  
//
//  Supports sending Graupner HoTT telemetry to transmitters that support it.
//
//  Can be used on any serial port (see UserConfiguration.h).
//
//  On AeroQuad32 only the TX pin needs to be cabled to the telemetry pin of the receiver. On Arduino based boards,
//  cable the telemetry pin to TX and RX (3.3V based boards). On 5V Arduinos a 1.8K resistor needs to be added at the TX pin.
//

#ifndef _AEROQUAD_TELEMETRY_GRAUPNER_HOTT_V4_H_
#define _AEROQUAD_TELEMETRY_GRAUPNER_HOTT_V4_H_

#include <Arduino.h>
#include "pins_arduino.h"
#include "GlobalDefined.h"
#include "Receiver.h"
#include "Motors.h"
#include "Compass.h"
#include "Kinematics.h"
#include "BarometricSensor.h"

#if GraupnerHoTT_SerialPort == 1
#define HOTT_SERIAL Serial1
#define STM32_USART USART1
#endif
#if GraupnerHoTT_SerialPort == 2
#define HOTT_SERIAL Serial2
#define STM32_USART USART2
#endif
#if GraupnerHoTT_SerialPort == 3
#define HOTT_SERIAL Serial3
#define STM32_USART USART3
#endif

#define HOTT_Rad2Deg 57.2957795

#define HOTT_VARIO_ID            0x89
#define HOTT_GPS_ID              0x8A
#define HOTT_ELECTRIC_AIR_ID     0x8E
#define HOTT_GENERAL_ID          0x8D

#define HOTT_MODE_BINARY    0
#define HOTT_MODE_TEXT     1

struct HoTT {
    unsigned int mode;      // Text mode or binary mode
    byte* msgBuffer;        // Current message buffer
    unsigned int msgLen;    // Remaining message length to be sent
    bool isSending;         // True while sending
    bool isFirstByte;       // True before sending first byte
    unsigned long lastByteMicros;   // Timestamp when last byte was sent
    int16_t msgCrc;             // Message CRC checksum
} HoTT;

/*
 HoTT General Message
 
 WarnBeep beeps and runs speech output:
 Q    Min cell voltage sensor 1
 R    Min Battery 1 voltage sensor 1
 J    Max Battery 1 voltage sensor 1
 F    Min temperature sensor 1
 H    Max temperature sensor 1
 S    Min Battery 2 voltage sensor 2
 K    Max Battery 2 voltage sensor 2
 G    Min temperature sensor 2
 I    Max temperature sensor 2
 W    Max current
 V    Max capacity mAh
 P    Min main power voltage
 X    Max main power voltage
 O    Min altitude
 Z    Max altitude
 C    negative difference m/s too high
 A    negative difference m/3s too high
 N    positive difference m/s too high
 L    positive difference m/3s too high
 T    Minimum RPM
 Y    Maximum RPM
 
 
 InverseStatus displays certain values in inverse font (white on black) for warning purposes.
 
 InverseStatus1 (Bitmask):
 0    all cell voltage
 1    Battery 1
 2    Battery 2
 3    Temperature 1
 4    Temperature 2
 5    Fuel
 6    mAh
 7    Altitude
 
 InverseStatus2 (Bitmask):
 0    main power current
 1    main power voltage
 2    Altitude
 3    m/s
 4    m/3s
 5    unknown
 6    unknown
 7    "ON" sign/text msg active 
 */
struct HoTTGeneral_t
{
    byte StartByte;               // #01 - 0x7C
    byte Packet_ID;               // #02 - HOTT_GENERAL_ID (0x8D)
    byte WarnBeep;                // #03 - Warning Beeps / Speech Output
    byte SensorID;                // #04 - 0xD0
    byte InverseStatus1;          // #05
    byte InverseStatus2;          // #06
    unsigned char VoltageCell1;   // #07 208 = 4,16V  (Voltage * 50 = Value)
    unsigned char VoltageCell2;   // #08 209 = 4,18V
    unsigned char VoltageCell3;   // #09
    unsigned char VoltageCell4;   // #10
    unsigned char VoltageCell5;   // #11
    unsigned char VoltageCell6;   // #12
    uint16_t  Battery1;           // #13-14 51 = 5,1V
    uint16_t  Battery2;           // #15-16 51 = 5,1V
    unsigned char Temperature1;   // #17 44 = 24°C, 0 = -20°C
    unsigned char Temperature2;   // #18 44 = 24°C, 0 = -20°C
    unsigned char FuelPercent;    // #19
    uint16_t  FuelCapacity;       // #20-21
    uint16_t  Rpm;                // #22-23
    uint16_t  Altitude;           // #24-25
    uint16_t  m_sec;              // #26-27 30000 = 0
    unsigned char m_3sec;         // #28 120 = 0
    uint16_t  Current;            // #29-30 1 = 0.1A
    uint16_t  InputVoltage;       // #31-32 66  = 6,6V
    uint16_t  Capacity;           // #33-34 1  = 10mAh
    uint16_t  Speed;              // #35-36
    unsigned char LowestCellVoltage;   // #37
    unsigned char LowestCellNumber;    // #38
    uint16_t  Rpm2;               // #39-40
    unsigned char ErrorNumber;    // #41
    unsigned char Pressure;       // #42  20=2,0bar
    byte Version;                 // #43 - 0x00
    byte EndByte;                 // #44 - 0x7D
} __attribute__((packed)) HoTTGeneral;

/*
 HoTT Electric Air Message
 
 WarnBeep:
 Q    Min cell voltage sensor 1
 R    Min Battery 1 voltage sensor 1
 J    Max Battery 1 voltage sensor 1
 F    Mim temperature sensor 1
 H    Max temperature sensor 1
 S    Min cell voltage sensor 2
 K    Max cell voltage sensor 2
 G    Min temperature sensor 2
 I    Max temperature sensor 2
 W    Max current
 V    Max capacity mAh
 P    Min main power voltage
 X    Max main power voltage
 O    Min altitude
 Z    Max altitude
 C    (negative) sink rate m/sec to high
 B    (negative) sink rate m/3sec to high
 N    climb rate m/sec to high
 M    climb rate m/3sec to high
 
 InverseStatus1:
 0    mAh
 1    Battery 1
 2    Battery 2
 3    Temperature 1
 4    Temperature 2
 5    Altitude
 6    Current
 7    Main power voltage
 
 InverseStatus2:
 0    m/s
 1    m/3s
 2    Altitude (duplicate?)
 3    m/s     (duplicate?)
 4    m/3s (duplicate?)
 5    unknown/unused
 6    unknown/unused
 7    "ON" sign/text msg active
 */
struct HoTTElectricAir_t
{
    byte StartByte;               // #01 - 0x7C
    byte Packet_ID;               // #02 - HOTT_ELECTRIC_AIR_ID (0x8E)
    byte WarnBeep;                // #03 - Warning Beeps / Speech Output
    byte SensorID;                // #04 - 0xE0
    byte InverseStatus1;          // #05
    byte InverseStatus2;          // #06
    unsigned char VoltageCell1;   // #07 208 = 4,16V  (Voltage * 50 = Value)
    unsigned char VoltageCell2;   // #08 209 = 4,18V
    unsigned char VoltageCell3;   // #09
    unsigned char VoltageCell4;   // #10
    unsigned char VoltageCell5;   // #11
    unsigned char VoltageCell6;   // #12
    unsigned char VoltageCell7;   // #13
    unsigned char VoltageCell8;   // #14
    unsigned char VoltageCell9;   // #15
    unsigned char VoltageCell10;  // #16
    unsigned char VoltageCell11;  // #17
    unsigned char VoltageCell12;  // #18
    unsigned char VoltageCell13;  // #19
    unsigned char VoltageCell14;  // #20
    uint16_t  Battery1;           // #21-22 51 = 5,1V
    uint16_t  Battery2;           // #23-24 51  = 5,1V
    unsigned char Temperature1;   // #25    44 = 24°C, 0 = -20°C
    unsigned char Temperature2;   // #26    44 = 24°C, 0 = -20°C
    uint16_t  Altitude;           // #27-28 500 = 0m
    uint16_t  Current;            // #29-30 1 = 0.1A
    uint16_t  InputVoltage;       // #31-32 66  = 6,6V
    uint16_t  Capacity;           // #33-34 1  = 10mAh
    uint16_t  m_sec;              // #35-36 30000 = 0
    unsigned char m_3sec;         // #37 120 = 0
    uint16_t  Rpm;                // #38-39
    unsigned char FlightTimeMinutes; // #40
    unsigned char FlightTimeSeconds; // #41
    unsigned char Speed;          // #42 - 1=2km
    byte Version;                 // #43 - 0x00
    byte EndByte;                 // #44 - 0x7D
} __attribute__((packed)) HoTTElectricAir;


/*
 HoTT Vario Message
 
 WarnBeep:
 Q    Min cell voltage sensor 1
 R    Min Battery 1 voltage sensor 1
 J    Max Battery 1 voltage sensor 1
 F    Min temperature sensor 1
 H    Max temperature sensor 1
 S    Min Battery voltage sensor 2
 K    Max Battery voltage sensor 2
 G    Min temperature sensor 2
 I    Max temperature sensor 2
 W    Max current
 V    Max capacity mAh
 P    Min main power voltage
 X    Max main power voltage
 O    Min altitude
 Z    Max altitude
 T    Minimum RPM
 Y    Maximum RPM
 C    m/s negative difference
 A    m/3s negative difference
 
 InverseStatus:
 TODO
 
 */
struct HoTTVario_t
{
    byte StartByte;               // #01 - 0x7C
    byte Packet_ID;               // #02 - HOTT_VARIO_ID (0x89)
    byte WarnBeep;                // #03 - Warning Beeps / Speech Output
    byte SensorID;                // #04 - 0x90
    byte InverseStatus;           // #05
    uint16_t Altitude;            // #06-07    // 500 = 0m
    uint16_t MaxAltitude;         // #08-09    // 500 = 0m
    uint16_t MinAltitude;         // #10-11    // 500 = 0m
    uint16_t m_sec;               // #12-13    // 3000 = 0
    uint16_t  m_3sec;             // #14-15    // 120 = 0
    uint16_t  m_10sec;            // #16-17
    char              Text[21];   // #18-38
    char              FreeCharacters[3]; // #39-41
    byte NullByte;                // #42 - 0x00
    byte Version;                 // #43 - 0x00
    byte EndByte;                 // #44 - 0x7D
} __attribute__((packed)) HoTTVario;

/*
 HoTT GPS Message
 
 WarnBeep:
 A    Min Speed
 L    Max Speed
 O    Min Altitude
 Z    Max Altitude
 C    (negative) sink rate m/sec to high
 B    (negative) sink rate m/3sec to high
 N    climb rate m/sec to high
 M    climb rate m/3sec to high
 D    Max home distance
 
 InverseStatus1:
 TODO
 
 InverseStatus2:
 1    No GPS Fix
 TODO
 
 */
struct HoTTGPS_t
{
    byte StartByte;               // #01 - 0x7C
    byte Packet_ID;               // #02 - HOTT_GPS_ID (0x8A)
    byte WarnBeep;                // #03 - Warning Beeps / Speech Output
    byte SensorID;                // #04 - 0xA0
    byte InverseStatus1;          // #05
    byte InverseStatus2;          // #06
    unsigned char Heading;        // #07     // 1 = 2°
    uint16_t Speed;               // #08-09   // in km/h
    unsigned char Lat_North;      // #10
    uint16_t Lat_G_M;             // #11-12
    uint16_t Lat_Sek;             // #13-14
    unsigned char Lon_East;       // #15
    uint16_t Lon_G_M;             // #16-17
    uint16_t Lon_Sek;             // #18-19
    uint16_t Distance;            // #20-21
    uint16_t Altitude;            // #22-23    // 500 = 0m
    uint16_t m_sec;               // #24-25    // 3000 = 0
    unsigned char m_3sec;         // #26 120 = 0
    unsigned char NumOfSats;      // #27
    unsigned char SatFix;         // #28 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix
    unsigned char HomeDirection;  // #29
    unsigned char AngleX;         // #30
    unsigned char AngleY;         // #31
    unsigned char AngleZ;         // #32
    uint32_t UtcTime;             // #33-36
    uint16_t SeaLevelAltitude;    // #37-38
    unsigned char Vibration;      // #39
    char FreeCharacters[3];       // #40-42
                                  // Char 3: 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix
    byte Version;                 // #43: 255 = Mikrokopter
    byte EndByte;                 // #44 - 0x7D
} __attribute__((packed)) HoTTGPS;


struct HoTTASCII_t
{
    byte StartByte;      // #001 - 0x7B
    byte Packet_ID;      // #002 - 0x00
    byte WarnBeep;       // #003 - Warning Beeps / Speech Output
    char Text[8*21];     // #004-172
    byte EndByte;        // #173 - 0x7D
} __attribute__((packed)) HoTTASCII;


/**
 * Enables RX and disables TX.
 */
static void hottV4EnableReceiverMode() {
#ifndef AeroQuadSTM32
    #if GraupnerHoTT_SerialPort == 1
        UCSR1B &= ~_BV(TXEN1);
        UCSR1B |= _BV(RXEN1);
    #endif
    #if GraupnerHoTT_SerialPort == 2
        UCSR2B &= ~_BV(TXEN2);
        UCSR2B |= _BV(RXEN2);
    #endif
    #if GraupnerHoTT_SerialPort == 3
        UCSR3B &= ~_BV(TXEN3);
        UCSR3B |= _BV(RXEN3);
    #endif
#endif
}

/**
 * Enabels TX and disables RX.
 */
static void hottV4EnableTransmitterMode() {
#ifndef AeroQuadSTM32
    #if GraupnerHoTT_SerialPort == 1
        UCSR1B &= ~_BV(RXEN1);
        UCSR1B |= _BV(TXEN1);
    #endif
    #if GraupnerHoTT_SerialPort == 2
        UCSR2B &= ~_BV(RXEN2);
        UCSR2B |= _BV(TXEN2);
    #endif
    #if GraupnerHoTT_SerialPort == 3
        UCSR3B &= ~_BV(RXEN3);
        UCSR3B |= _BV(TXEN3);
    #endif
#endif
}

void prepareHoTTGeneral() {
    HoTT.lastByteMicros = micros();     // for initial delay
    HoTT.msgBuffer = (byte*) &HoTTGeneral;
    HoTT.msgLen = sizeof(HoTTGeneral);
    HoTT.isSending = true;
    HoTT.isFirstByte = true;
    
    HoTTGeneral.InverseStatus1 = 0;
    HoTTGeneral.InverseStatus2 = 0;
    HoTTGeneral.WarnBeep = ' ';
    
    // Battery
    HoTTGeneral.Battery1 = batteryData[0].voltage/10.0;
    HoTTGeneral.Battery2 = batteryData[0].voltage/10.0;
    HoTTGeneral.InputVoltage = batteryData[0].voltage/10.0;
    
    if (batteryAlarm) {
        HoTTGeneral.WarnBeep = 'R';
    }
    
    if (batteryAlarm || batteryWarning) {
        HoTTGeneral.InverseStatus1 |= (1 << 1);
        HoTTGeneral.InverseStatus1 |= (1 << 2);
        HoTTGeneral.InverseStatus2 |= (1 << 1);
    }
    
    
    // Altitude
    int altitude = getBaroAltitude();
    HoTTGeneral.Altitude = altitude + 500;
    
    // Motors Armed
    if (motorArmed == ON) {
        HoTTGeneral.InverseStatus2 |= (1 << 7);
    }
    
    // Show receiver commands as cell voltages (125 = 2,5V = center)
    HoTTGeneral.VoltageCell1 = (receiverCommand[0] - 1000) / 4;
    HoTTGeneral.VoltageCell2 = (receiverCommand[1] - 1000) / 4;
    HoTTGeneral.VoltageCell3 = (receiverCommand[2] - 1000) / 4;
    HoTTGeneral.VoltageCell4 = (receiverCommand[3] - 1000) / 4;
    HoTTGeneral.VoltageCell5 = (receiverCommand[4] - 1000) / 4;
    HoTTGeneral.VoltageCell6 = (receiverCommand[5] - 1000) / 4;

}

void prepareHoTTVario() {
    HoTT.lastByteMicros = micros();     // for initial delay
    HoTT.msgBuffer = (byte*) &HoTTVario;
    HoTT.msgLen = sizeof(HoTTVario);
    HoTT.isSending = true;
    HoTT.isFirstByte = true;

    
    // Kinematics Angle as Altitudes
    HoTTVario.Altitude = (kinematicsAngle[0] * HOTT_Rad2Deg) + 500;
    HoTTVario.MaxAltitude = (kinematicsAngle[1] * HOTT_Rad2Deg) + 500;
    HoTTVario.MinAltitude = (kinematicsAngle[2] * HOTT_Rad2Deg) + 500;

    // Flight Mode
    if (flightMode == ATTITUDE_FLIGHT_MODE) {
        memcpy(HoTTVario.Text, "Attitud", 7);
    } else if (flightMode == RATE_FLIGHT_MODE) {
        memcpy(HoTTVario.Text, "Rate   ", 7);
    }
    
    // Altitude Hold State
#ifdef UseGpsNavigator
    if (positionHoldState == ON && altitudeHoldState == ON) {
        memcpy(HoTTVario.Text+7, "PosHold", 7);
    } else {
#endif
        if (altitudeHoldState == ON) {
            memcpy(HoTTVario.Text+7, "AltHold", 7);
        } else if (altitudeHoldState == ALTPANIC) {
            memcpy(HoTTVario.Text+7, "AltPani", 7);
        } else {
            memcpy(HoTTVario.Text+7, "       ", 7);
        }
#ifdef UseGpsNavigator
    }
#endif

}

void prepareHoTTElectricAir() {
    HoTT.lastByteMicros = micros();     // for initial delay
    HoTT.msgBuffer = (byte*) &HoTTElectricAir;
    HoTT.msgLen = sizeof(HoTTElectricAir);
    HoTT.isSending = true;
    HoTT.isFirstByte = true;
    
    // Battery
    HoTTElectricAir.Battery1 = batteryData[0].voltage/10.0;
    HoTTElectricAir.Battery2 = batteryData[0].voltage/10.0;
    HoTTElectricAir.InputVoltage = batteryData[0].voltage/10.0;

    // Altitude
    int altitude = getBaroAltitude();
    HoTTElectricAir.Altitude = altitude + 500;
    
    // Gyro as cell voltages 1-3 (125 = 2.5V = 0.0), resolution 0.01 = 0.1
    HoTTElectricAir.VoltageCell1 = (gyroRate[0] * 5) + 125;
    HoTTElectricAir.VoltageCell2 = (gyroRate[1] * 5) + 125;
    HoTTElectricAir.VoltageCell3 = (gyroRate[2] * 5) + 125;
    
    // Accelerometer as cell voltages 4-6 (125 = 2.5V = 0.0), resolution 0.01 = 0.1 m/s^2
    HoTTElectricAir.VoltageCell4 = (meterPerSecSec[0] * 5) + 125;
    HoTTElectricAir.VoltageCell5 = (meterPerSecSec[1] * 5) + 125;
    HoTTElectricAir.VoltageCell6 = (meterPerSecSec[2] * 5) + 125;
    
    // Magnetometer as cell voltages 8-10 (125 = 2,5V = 0.0), resolution 0.01 = 2 deg)
    HoTTElectricAir.VoltageCell8 = (getMagnetometerRawData(0) / 4) + 125;
    HoTTElectricAir.VoltageCell9 = (getMagnetometerRawData(1) / 4) + 125;
    HoTTElectricAir.VoltageCell10 = (getMagnetometerRawData(2) / 4) + 125;
    
    // Motor Commands as cell voltages 11-14 (125 = 2,5V = center)
    HoTTElectricAir.VoltageCell11 = (motorCommand[0] - 1000) / 4;
    HoTTElectricAir.VoltageCell12 = (motorCommand[1] - 1000) / 4;
    HoTTElectricAir.VoltageCell13 = (motorCommand[2] - 1000) / 4;
    HoTTElectricAir.VoltageCell14 = (motorCommand[3] - 1000) / 4;
}

void convertGpsCoordinate(int32_t coord, unsigned char* p_direction, uint16_t* p_degMin, uint16_t* p_sek) {
    float coordFloat = abs(coord / 10000000.0); // Convert to decimal
    
    int degMin = coordFloat; // Degrees
    coordFloat -= degMin;
    coordFloat *= 60; // minutes
    int min = coordFloat;
    degMin = degMin * 100 + min;
    
    coordFloat -= min;
    
    *p_direction = (coord < 0.0);
    *p_degMin = degMin;
    *p_sek = coordFloat * 10000;
}

void prepareHoTTGPS() {
    HoTT.lastByteMicros = micros();     // for initial delay
    HoTT.msgBuffer = (byte*) &HoTTGPS;
    HoTT.msgLen = sizeof(HoTTGPS);
    HoTT.isSending = true;
    HoTT.isFirstByte = true;
        
#ifdef UseGPS
    HoTTGPS.NumOfSats = gpsData.sats;
    
    if (gpsData.state == GPS_DETECTING || gpsData.state == GPS_NOFIX) {
        HoTTGPS.InverseStatus2 = 1;
        HoTTGPS.SatFix = '-';
    } else if (gpsData.state == GPS_FIX2D) {
        HoTTGPS.InverseStatus2 = 0;
        HoTTGPS.SatFix = '2';
    } else if (gpsData.state == GPS_FIX3D) {
        HoTTGPS.InverseStatus2 = 0;
        HoTTGPS.SatFix = '3';
    } else if (gpsData.state == GPS_FIX3DD) {
        HoTTGPS.InverseStatus2 = 0;
        HoTTGPS.SatFix = 'D';
    }
    HoTTGPS.FreeCharacters[2] = HoTTGPS.SatFix;

    convertGpsCoordinate(gpsData.lat, &HoTTGPS.Lat_North, &HoTTGPS.Lat_G_M, &HoTTGPS.Lat_Sek);
    convertGpsCoordinate(gpsData.lon, &HoTTGPS.Lon_East, &HoTTGPS.Lon_G_M, &HoTTGPS.Lon_Sek);
    
    HoTTGPS.Heading = gpsData.course / 100000 / 2;
    HoTTGPS.Speed = gpsData.speed / 100 / 1000 / 3600; // convert from cm/s to km/h
    HoTTGPS.Altitude = getBaroAltitude() + 500;
    HoTTGPS.SeaLevelAltitude = gpsData.height / 10 / 100; // convert from mm to m
    
    HoTTGPS.AngleX = kinematicsAngle[XAXIS] * HOTT_Rad2Deg / 2;
    HoTTGPS.AngleY = kinematicsAngle[YAXIS] * HOTT_Rad2Deg / 2;
    HoTTGPS.AngleZ = kinematicsAngle[ZAXIS] * HOTT_Rad2Deg / 2;
    
    HoTTGPS.UtcTime = gpsData.fixtime;
#endif

}


/*
 * Initializes Telemetry.
 *
 * Used to set some default values.
 */
void initializeTelemetry() {
    HOTT_SERIAL.begin(19200);
#ifdef AeroQuadSTM32
    // Configure Half Duplex Mode, see section 26.3.10 of Reference Manual
    STM32_USART->regs->CR3 |= USART_CR3_HDSEL;
#endif
    hottV4EnableReceiverMode();
    
    // Init HoTTGeneral
    HoTTGeneral.StartByte = 0x7C;
    HoTTGeneral.Packet_ID = HOTT_GENERAL_ID;
    HoTTGeneral.SensorID = 0xD0;
    HoTTGeneral.EndByte = 0x7D;
    HoTTGeneral.Temperature1 = 20;
    HoTTGeneral.Temperature2 = 20;
    HoTTGeneral.m_sec = 30000;
    HoTTGeneral.m_3sec = 120;
    
    // Init Vario
    HoTTVario.StartByte = 0x7C;
    HoTTVario.Packet_ID = HOTT_VARIO_ID;
    HoTTVario.SensorID = 0x90;
    HoTTVario.EndByte = 0x7D;
    HoTTVario.Altitude = 500;
    HoTTVario.MaxAltitude = 500;
    HoTTVario.MinAltitude = 500;
    
    // Init Electric Air
    HoTTElectricAir.StartByte = 0x7C;
    HoTTElectricAir.Packet_ID = HOTT_ELECTRIC_AIR_ID;
    HoTTElectricAir.SensorID = 0xE0;
    HoTTElectricAir.EndByte = 0x7D;
    HoTTElectricAir.Temperature1 = 20;
    HoTTElectricAir.Temperature2 = 20;
    HoTTElectricAir.Altitude = 500;
    HoTTElectricAir.m_sec = 30000;
    HoTTElectricAir.m_3sec = 120;
    
    // Init GPS
    HoTTGPS.StartByte = 0x7C;
    HoTTGPS.Packet_ID = HOTT_GPS_ID;
    HoTTGPS.SensorID = 0xA0;
    HoTTGPS.EndByte = 0x7D;
    HoTTGPS.Altitude = 500;
    HoTTGPS.m_sec = 30000;
    HoTTGPS.m_3sec = 120;
    HoTTGPS.Version = 255;
}

/*
 * This cares for sending the current telemetry packet.
 *
 * Timing of HoTT is a litte difficult. It requires a 2-3ms pause between
 * each byte and an initial delay of 5ms. Therefore the send is in the main
 * loop() and just ensures to continue sending without interrupting the flight
 * processing.
 */
void sendTelemetry() {
    if (!HoTT.isSending) return;

    long tNow = micros();
    
    if (tNow - HoTT.lastByteMicros < 2500) return;   // ensure 2ms delay between bytes (2-3ms required)
    if (HoTT.isFirstByte && (tNow - HoTT.lastByteMicros < 4500)) return; // ensure 4ms delay before first byte (5ms required)
    
    if (HoTT.isFirstByte) {
        if (HOTT_SERIAL.available()) {
            // It was other sensors data, don't send now
            while (HOTT_SERIAL.read() != -1); // Clear out receive buffer
            HoTT.isSending = false;
            return;
        }
        
        hottV4EnableTransmitterMode();
        HoTT.msgCrc = 0;
        HoTT.isFirstByte = false;
    }
    
    if (HoTT.msgLen > 0) {
        HoTT.msgCrc += *(HoTT.msgBuffer);
        HOTT_SERIAL.write(*(HoTT.msgBuffer));
        HoTT.lastByteMicros = tNow;
        
        HoTT.msgLen--;
        HoTT.msgBuffer++;
    } else {
        // Send checksum
        HOTT_SERIAL.write((byte) HoTT.msgCrc);
        
        // Done with this message
        HoTT.isSending = false;
        while (HOTT_SERIAL.read() != -1); // Clear out receive buffer
        hottV4EnableReceiverMode();
    }
    
    
    
}

/*
 * Reads a new telemetry request from the serial line.
 */
void processTelemetryCommand() {
    if (HoTT.isSending) return;
    
    while (HOTT_SERIAL.available()) {
        uint8_t inByte = HOTT_SERIAL.read();

        if (inByte == 0x80) {
            HoTT.mode = HOTT_MODE_BINARY;
        } else if (inByte == 0x7F) {
            HoTT.mode = HOTT_MODE_TEXT;
        }
        
        if (HoTT.mode == HOTT_MODE_BINARY) {
            if (inByte == HOTT_GENERAL_ID) {
                prepareHoTTGeneral();
            } else if (inByte == HOTT_VARIO_ID) {
                prepareHoTTVario();
            } else if (inByte == HOTT_ELECTRIC_AIR_ID) {
                prepareHoTTElectricAir();
            } else if (inByte == HOTT_GPS_ID) {
                prepareHoTTGPS();
            }
        }
        
        // We are sending now, don't read further command bytes
        if (HoTT.isSending) return;
    }
}



#endif
