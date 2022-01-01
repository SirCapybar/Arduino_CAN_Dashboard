#include <mcp_can.h>
#include <SPI.h>

#define SPI_CS_PIN 10
#define CAN_ID_MODE MCP_ANY
#define CAN_SPEED CAN_500KBPS
#define CAN_CLOCK MCP_8MHZ
#define CAN_REFRESH_RATE_US 50000UL

#define HANDBRAKE_CONTROL_PIN 9
#define FOG_LIGHT_INDICATOR_PIN 8
#define HIGH_BEAM_INDICATOR_PIN 7

#define MAX_RPM 6000U
#define MAX_KMH 240U

struct DashboardSettings
{
    unsigned short speed = 0;         // speed [km/h]
    unsigned short rpm = 0;           // revs [rpm]
    bool backlight = true;            // dashboard backlight (on/off)
    byte turning_lights = 0;          // turning lights (0: off, 1: left, 2: right, 3: both)
    bool abs = false;                 // ABS lamp (on/off)
    bool offroad = false;             // offroad lamp (on/off)
    bool handbrake = false;           // handbrake lamp (on/off)
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
unsigned long last_can_us = 0UL;

void canSend(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h)
{
    unsigned char data[8] = {a, b, c, d, e, f, g, h};
    can.sendMsgBuf(address, 0, 8, data);
}

void sendImmobilizer()
{
    canSend(0x3D0, 0, 0x80, 0, 0, 0, 0, 0, 0);
}

void sendEngineOnAndESP()
{
    canSend(0xDA0, 0x01, 0x80, 0, 0, 0, 0, 0, 0);
}

void sendSpeedRPMDriveMode()
{
    int speed = static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.007f);
    byte speedLow = speed & 0xFF, speedHigh = (speed >> 8) & 0xFF;
    // motorspeed
    canSend(0x320, 0, speedLow, speedHigh, 0, 0, 0, 0, 0);
    // rpm
    unsigned short rpm = dashboard.rpm * 4;
    canSend(0x280, 0x49, 0x0E, rpm & 0xFF, (rpm >> 8) & 0xFF, 0, 0, 0, 0);
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
    canSend(0x5A0, 0xFF, speedLow, speedHigh, drive_mode, 0, 0, 0, 0xAD);
    digitalWrite(HANDBRAKE_CONTROL_PIN, dashboard.handbrake ? LOW : HIGH);
    // for ABS
    canSend(0x1A0, 0x18, speedLow, speedHigh, 0, 0xFE, 0xFE, 0, 0xFF);
}

void sendSeatBeltWarning()
{
    // airbag and seat belt info
    byte seat_belt = 0;
    if (dashboard.seat_belt_warning)
    {
        seat_belt = 0x04;
    }
    canSend(0x050, 0, 0x80, seat_belt, 0, 0, 0, 0, 0);
}

void sendEngineControl()
{
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
}

void sendLights()
{
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
}

void updateDashboard()
{
    unsigned long us = micros();
    unsigned long diff = us - last_can_us;
    if (diff >= CAN_REFRESH_RATE_US)
    {
        last_can_us += diff / CAN_REFRESH_RATE_US * CAN_REFRESH_RATE_US;
        sendImmobilizer();
        sendEngineOnAndESP();
        sendLights();
        sendEngineControl();
        sendSpeedRPMDriveMode();
        sendSeatBeltWarning();
    }
}

void setup()
{
    pinMode(HANDBRAKE_CONTROL_PIN, OUTPUT);
    pinMode(FOG_LIGHT_INDICATOR_PIN, OUTPUT);
    pinMode(HIGH_BEAM_INDICATOR_PIN, OUTPUT);

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
}

void loop()
{
    updateDashboard();
    if (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
        case 'a':
            dashboard.backlight = !dashboard.backlight;
            break;
        case 'b':
            dashboard.abs = !dashboard.abs;
            break;
        case 'c':
            dashboard.offroad = !dashboard.offroad;
            break;
        case 'd':
            dashboard.handbrake = !dashboard.handbrake;
            break;
        case 'e':
            dashboard.low_tire_pressure = !dashboard.low_tire_pressure;
            break;
        case 'f':
            dashboard.door_open_warning = !dashboard.door_open_warning;
            break;
        case 'g':
            dashboard.clutch_control = !dashboard.clutch_control; // "EMBRAY"
            break;
        case 'h':
            dashboard.check_lamp = !dashboard.check_lamp;
            break;
        case 'i':
            dashboard.trunk_open_warning = !dashboard.trunk_open_warning;
            break;
        case 'j':
            dashboard.battery_warning = !dashboard.battery_warning;
            break;
        case 'k':
            dashboard.key_battery_warning = !dashboard.key_battery_warning; // "PILECLE"
            break;
        case 'l':
            dashboard.fog_light = !dashboard.fog_light;
            break;
        case 'm':
            dashboard.high_beam = !dashboard.high_beam;
            break;
        case 'n':
            dashboard.seat_belt_warning = !dashboard.seat_belt_warning;
            break;
        case 'o':
            dashboard.preheating = !dashboard.preheating;
            break;
        case 'p':
            dashboard.high_water_temp = !dashboard.high_water_temp;
            break;
        case 'q':
            dashboard.dpf_warning = !dashboard.dpf_warning;
            break;
        case 'r':
            dashboard.turning_lights = dashboard.turning_lights == 3 ? 0 : (dashboard.turning_lights + 1);
            break;
        case 's':
            dashboard.speed += 50;
            if (dashboard.speed > MAX_KMH)
            {
                dashboard.speed = 0;
            }
            break;
        case 't':
            dashboard.rpm += 500;
            if (dashboard.rpm > MAX_RPM)
            {
                dashboard.rpm = 0;
            }
            break;
        }
    }
}
