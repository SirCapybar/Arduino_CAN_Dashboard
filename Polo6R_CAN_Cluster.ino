#include <mcp_can.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define SPI_CS_PIN 10
#define CAN_ID_MODE MCP_ANY
#define CAN_SPEED CAN_500KBPS
#define CAN_CLOCK MCP_8MHZ

#define PARKING_BRAKE_CONTROL_PIN 9
#define FOG_LIGHT_INDICATOR_PIN 8
#define HIGH_BEAM_INDICATOR_PIN 7

#define MICROS_PER_UPDATE 100000UL // 10Hz

#define MAX_RPM 6000U
#define MAX_KMH 240U

#define MSG_DELIMITER ';'

#define RESCALE_RPM
#define RESCALE_KMH

/*
    Usually supported:
    DataCorePlugin.GamePaused
    DataCorePlugin.GameData.Rpms
    DataCorePlugin.GameData.MaxRpm
    DataCorePlugin.GameData.SpeedKmh
    DataCorePlugin.GameRawData.Physics.Abs ??
    DataCorePlugin.GameData.Handbrake

    DataCorePlugin.GameData.Gear -> R,N,1,2,3,...
    DataCorePlugin.ExternalScript.BlinkingGearUP -> same as gear but changes to UP when upshift needed
    DataCorePlugin.GameData.CarSettings_RPMRedLineReached -> 0/1
    DataCorePlugin.GameData.CarSettings_RPMShiftLight1 -> 0.0-1.0
    DataCorePlugin.GameData.CarSettings_RPMShiftLight2 -> 0.0-1.0

    Supported by ETS2:
    DataCorePlugin.GameRawData.TruckValues.CurrentValues.LightsValues.DashboardBacklight
    DataCorePlugin.GameRawData.TruckValues.CurrentValues.MotorValues.BrakeValues.ParkingBrake
    DataCorePlugin.GameRawData.TruckValues.CurrentValues.EngineEnabled
    DataCorePlugin.GameRawData.TruckValues.CurrentValues.LightsValues.BeamLow
    DataCorePlugin.GameRawData.TruckValues.CurrentValues.LightsValues.BeamHigh
    DataCorePlugin.GameData.TurnIndicatorLeft
    DataCorePlugin.GameData.TurnIndicatorRight
    DataCorePlugin.GameRawData.TruckValues.CurrentValues.DashboardValues.WarningValues.BatteryVoltage
    DataCorePlugin.GameRawData.TruckValues.CurrentValues.DashboardValues.WarningValues.WaterTemperature
*/

struct DashboardSettings
{
    unsigned short speed = 0;         // speed [km/h]
    unsigned short rpm = 0;           // revs [rpm]
    bool backlight = true;            // dashboard backlight (off/on)
    byte turning_lights = 0;          // turning lights (0: off, 1: left, 2: right, 3: both)
    bool abs = false;                 // ABS lamp (on/off)
    bool offroad = false;             // offroad lamp (on/off)
    bool handbrake = false;           // handbrake lamp (on/off)
    bool parking_brake = false;       // parking brake lamp (on/off)
    bool low_tire_pressure = false;   // low tire pressure lamp (on/off)
    bool door_open_warning = false;   // open door warning lamp (on/off)
    bool clutch_control = false;      // display clutch message on dashboard display (on/off)
    bool check_lamp = false;          // display 'check lamp' message on dashboard display (on/off)
    bool trunk_open_warning = false;  // open trunk warning lamp (on/off)
    bool battery_warning = false;     // battery warning lamp (on/off)
    bool key_battery_warning = false; // display 'key battery low' message on dashboard display. probably works only at start (on/off)
    bool fog_light = false;           // fog light lamp (on/off)
    bool high_beam = false;           // high beam lamp (on/off)
    bool seat_belt_warning = false;   // seat belt warning lamp (on/off)
    bool preheating = false;          // diesel preheating lamp (on/off)
    bool high_water_temp = false;     // high water temperature lamp (on/off) AVOID IT PLEASE, BEEPS AWFULLY LOUD
    bool dpf_warning = false;         // DPF warning lamp (on/off)
};

MCP_CAN can(SPI_CS_PIN);
DashboardSettings dashboard;
unsigned long last_us;
bool even_interrupt = true;

void canSend(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h)
{
    unsigned char data[8] = {a, b, c, d, e, f, g, h};
    can.sendMsgBuf(address, 0, 8, data);
}

void setup()
{
    pinMode(PARKING_BRAKE_CONTROL_PIN, OUTPUT);
    pinMode(FOG_LIGHT_INDICATOR_PIN, OUTPUT);
    pinMode(HIGH_BEAM_INDICATOR_PIN, OUTPUT);

    // initialize CAN module and serial
    Serial.begin(115200);
    byte can_result = can.begin(CAN_ID_MODE, CAN_SPEED, CAN_CLOCK);
    while (can_result != CAN_OK)
    {
        Serial.println("CAN initialization failed! Retrying...");
        delay(250);
        can_result = can.begin(CAN_ID_MODE, CAN_SPEED, CAN_CLOCK);
    }
    Serial.println("CAN initialized!");
    can.setMode(MCP_NORMAL);
    last_us = micros();
}

void updateDashboard()
{
    // immobilizer
    canSend(0x3D0, 0, 0x80, 0, 0, 0, 0, 0, 0);

    // lights
    byte b0 = dashboard.turning_lights & 0x03;
    if (dashboard.battery_warning)
    {
        b0 |= 0x80;
    }
    byte b1 = 0;
    if (dashboard.door_open_warning)
    {
        b1 |= 0x01;
    }
    if (dashboard.trunk_open_warning)
    {
        b1 |= 0x20;
    }
    byte b2 = 0;
    if (dashboard.backlight)
    {
        b2 |= 0x01;
    }
    byte b4 = 0;
    if (dashboard.clutch_control)
    {
        b4 |= 0x01;
    }
    if (dashboard.check_lamp)
    {
        b4 |= 0x10;
    }
    byte b5 = 0;
    if (dashboard.key_battery_warning)
    {
        b5 |= 0x80;
    }
    byte b7 = 0;
    if (dashboard.fog_light)
    {
        b7 |= 0x20;
    }
    if (dashboard.high_beam)
    {
        b7 |= 0x40;
    }
    canSend(0x470, b0, b1, b2, 0, b4, b5, 0, b7);
    digitalWrite(FOG_LIGHT_INDICATOR_PIN, dashboard.fog_light ? HIGH : LOW);
    digitalWrite(HIGH_BEAM_INDICATOR_PIN, dashboard.high_beam ? HIGH : LOW);

    // engine control
    byte engine_control = 0;
    if (dashboard.preheating)
    {
        engine_control |= 0x02;
    }
    if (dashboard.high_water_temp)
    {
        engine_control |= 0x10;
    }
    byte dpf_warning = 0;
    if (dashboard.dpf_warning)
    {
        dpf_warning = 0x02;
    }
    canSend(0x480, 0, engine_control, 0, 0, 0, dpf_warning, 0, 0);

    int speed_prescaled = static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.007f);
    byte speed_low = speed_prescaled & 0xFF, speed_high = (speed_prescaled >> 8) & 0xFF;

    int abs_speed_prescaled = static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.01f);
    byte abs_speed_low = speed_prescaled & 0xFF, abs_speed_high = (speed_prescaled >> 8) & 0xFF;
    byte abs_speed15_low = (speed_prescaled << 1) & 0xFF, abs_speed15_high = (speed_prescaled >> 7) & 0xFF;

    // airbag and seat belt info
    byte seat_belt = 0;
    if (dashboard.seat_belt_warning)
    {
        seat_belt = 0x04;
    }
    canSend(0x050, 0, 0x80, seat_belt, 0, 0, 0, 0, 0);

    // engine on and ESP
    canSend(0xDA0, 0x01, 0x80, 0, 0, 0, 0, 0, 0);

    // motor speed? doesn't affect the dashboard at all
    canSend(0x320, 0x04, 0, 0x40, abs_speed_low, abs_speed_high, abs_speed_low, abs_speed_high, 0);

    // RPM
    unsigned short rpm = dashboard.rpm * 4;
    canSend(0x280, 0x49, 0x0E, rpm & 0xFF, (rpm >> 8) & 0xFF, 0, 0, 0, 0);

    // speed and drive mode
    byte drive_mode = 0;
    if (dashboard.abs)
    {
        drive_mode |= 0x01;
    }
    if (dashboard.offroad)
    {
        drive_mode |= 0x02;
    }
    if (dashboard.handbrake)
    {
        drive_mode |= 0x04;
    }
    if (dashboard.low_tire_pressure)
    {
        drive_mode |= 0x08;
    }
    // actual speed and drivemode, could contain mileage counter as byte 5 and 6 (0-indexed)
    canSend(0x5A0, 0xFF, speed_low, speed_high, drive_mode, 0, 0, 0, 0xAD);
    digitalWrite(PARKING_BRAKE_CONTROL_PIN, dashboard.parking_brake ? LOW : HIGH);

    // ABS1: has to be sent to apply the speed, though it can all be zeros?
    canSend(0x1A0, 0, 0x18, abs_speed15_low, abs_speed15_high, 0xFE, 0xFE, 0, 0x00);

    // ABS2: doesn't affect the dashboard?
    canSend(0x4A0, abs_speed15_low, abs_speed15_high, abs_speed15_low, abs_speed15_high, abs_speed15_low, abs_speed15_high, abs_speed15_low, abs_speed15_high);
}

int find_next_delimiter(const char *str)
{
    int counter = 0;
    do
    {
        if (*str == MSG_DELIMITER)
        {
            return counter;
        }
        ++str;
        ++counter;
    } while (*str != '\0');
    return -1;
}

int str2int(const char *str, int len)
{
    int ret = 0;
    for (int i = 0; i < len; ++i)
    {
        ret = ret * 10 + (str[i] - '0');
    }
    return ret;
}

bool readBool(const char *&str)
{
    bool result = *str == '1';
    str += 2;
    return result;
}

char readChar(const char *&str)
{
    char result = *str;
    str += 2;
    return result;
}

int readInt(const char *&str)
{
    int next_delimiter_ind = find_next_delimiter(str);
    int result = str2int(str, next_delimiter_ind);
    str += next_delimiter_ind + 1;
    return result;
}

void loop()
{
    unsigned long diff = micros() - last_us;
    if (diff >= MICROS_PER_UPDATE)
    {
        last_us += diff / MICROS_PER_UPDATE * MICROS_PER_UPDATE;
        updateDashboard();
    }
    if (Serial.available())
    {
        unsigned long start = micros();
        String line = Serial.readString();
        // Serial.println(line);
        const char *cstr = line.c_str();
        // PAUSE;RPM;MAX_RPM;SPEED;MAX_SPEED;ABS;HANDBRAKE;PARKING_BRAKE;TURN_LEFT;TURN_RIGHT;HIGH_BEAM;BATTERY_VOLTAGE;WATER_TEMPERATURE;BACKLIGHT;
        // Xbool;int;int;int;int;bool;bool;bool;bool;bool;bool;bool;bool;bool;
        // example: X1;500;1000;50;100;1;1;1;1;1;1;1;0;1;
        if (*cstr == 'X') // the message starts with an X
        {
            ++cstr;
            bool pause = readBool(cstr);
            dashboard.door_open_warning = pause;
            dashboard.trunk_open_warning = pause;
            int rpm = readInt(cstr);
            int max_rpm = readInt(cstr);
#ifdef RESCALE_RPM
            rpm = map(rpm, 0, max_rpm, 0, MAX_RPM);
#endif
            if (rpm > MAX_RPM)
            {
                rpm = MAX_RPM;
            }
            else if (rpm < 0)
            {
                rpm = 0;
            }
            dashboard.rpm = rpm;
            int speed = readInt(cstr);
            int max_speed = readInt(cstr);
#ifdef RESCALE_KMH
            speed = map(speed, 0, max_speed, 0, MAX_KMH);
#endif
            if (speed > MAX_KMH)
            {
                speed = MAX_KMH;
            }
            else if (speed < 0)
            {
                speed = 0;
            }
            dashboard.speed = speed;
            dashboard.abs = readBool(cstr);
            dashboard.handbrake = readBool(cstr);
            dashboard.parking_brake = readBool(cstr);
            bool turn_left = readBool(cstr);
            bool turn_right = readBool(cstr);
            if (turn_left)
            {
                dashboard.turning_lights = turn_right ? 3 : 1;
            }
            else
            {
                dashboard.turning_lights = turn_right ? 2 : 0;
            }
            dashboard.high_beam = readBool(cstr);
            dashboard.battery_warning = readBool(cstr);
            bool water_temperature_warning = readBool(cstr); // don't use this please, it's loud AF
            bool backlight = readBool(cstr);                 // I don't like disabling the backlight, keep it on
        }
        int diff = micros() - start;
        Serial.println(diff);
    }
}
