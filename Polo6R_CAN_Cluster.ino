#include <mcp_can.h>
#include <SPI.h>

#define SPI_CS_PIN 10
#define CAN_ID_MODE MCP_ANY
#define CAN_SPEED CAN_500KBPS
#define CAN_CLOCK MCP_16MHZ

#define lo8(x) ((int)(x)&0xff)
#define hi8(x) ((int)(x) >> 8)

#define HANDBRAKE_CONTROL_PIN 9
#define FOG_LIGHT_INDICATOR_PIN 8
#define HIGH_BEAM_INDICATOR_PIN 7

#define DEFINE_SETTER(setter_name, field_type, field_name, send_method_name) \
    void setter_name(field_type field_name)                                  \
    {                                                                        \
        dashboard.field_name = field_name;                                   \
        /* send_method_name(); */                                            \
    }

#define DEFINE_BSETTER(setter_name, field_name, send_method_name) \
    DEFINE_SETTER(setter_name, bool, field_name, send_method_name)

struct DashboardSettings
{
    unsigned short speed = 0;         // speed, enough precision for about 491 [km/h]
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
    bool seat_belt_warning = false;   // seat belt warning lamp (on/off) PLEASE DISABLE IT, IT BEEPS SO LOUDLY ;_;
    bool preheating = false;          // diesel preheating lamp (on/off)
    bool high_water_temp = false;     // high water temperature lamp (on/off)
    bool dpf_warning = false;         // DPF warning lamp (on/off)
};

MCP_CAN can(SPI_CS_PIN);

DashboardSettings dashboard;

const unsigned long CAN_REFRESH_RATE = 20UL;
const unsigned long CAN_LIGHT_ENGINE_REFRESH_RATE = 200UL;
unsigned long last_dashboard_millis = 0UL;
unsigned long last_can_millis = 0UL;
unsigned long last_can_light_engine_millis = 0UL;
bool is_set = false;

void setup()
{
    pinMode(HANDBRAKE_CONTROL_PIN, OUTPUT);
    pinMode(FOG_LIGHT_INDICATOR_PIN, OUTPUT);
    pinMode(HIGH_BEAM_INDICATOR_PIN, OUTPUT);
    digitalWrite(HANDBRAKE_CONTROL_PIN, !dashboard.handbrake ? HIGH : LOW);
    digitalWrite(FOG_LIGHT_INDICATOR_PIN, dashboard.fog_light ? HIGH : LOW);
    digitalWrite(HIGH_BEAM_INDICATOR_PIN, dashboard.high_beam ? HIGH : LOW);

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

    last_dashboard_millis = last_can_millis = last_can_light_engine_millis = millis();
}

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

void sendSpeedAndDriveMode()
{
    int speed = static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.0075f);
    byte speedLow = speed & 0xFF, speedHigh = (speed >> 8) & 0xFF;
    // motorspeed
    canSend(0x320, 0, speedLow * 100, speedHigh * 100, 0, 0, 0, 0, 0);
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
    // for ABS
    canSend(0x1A0, 0x18, speedLow, speedHigh, 0, 0xFE, 0xFE, 0, 0xFF);
}

void sendRPM()
{
    unsigned short rpm = dashboard.rpm * 4;
    canSend(0x280, 0x49, 0x0E, rpm & 0xFF, (rpm >> 8) & 0xFF, 0, 0, 0, 0);
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
}

DEFINE_SETTER(setSpeed, unsigned short, speed, sendSpeedAndDriveMode)
DEFINE_BSETTER(setABS, abs, sendSpeedAndDriveMode)
DEFINE_BSETTER(setOffroad, offroad, sendSpeedAndDriveMode)

void setHandbrake(bool handbrake)
{
    dashboard.handbrake = handbrake;
    digitalWrite(HANDBRAKE_CONTROL_PIN, !handbrake ? HIGH : LOW);
    // sendSpeedAndDriveMode();
}

DEFINE_BSETTER(setLowTirePressure, low_tire_pressure, sendSpeedAndDriveMode)
DEFINE_SETTER(setRPM, unsigned short, rpm, sendRPM)
DEFINE_BSETTER(setSeatBeltWarning, seat_belt_warning, sendSeatBeltWarning)
DEFINE_BSETTER(setPreheating, preheating, sendEngineControl)
DEFINE_BSETTER(setHighWaterTemperature, high_water_temp, sendEngineControl)
DEFINE_BSETTER(setDPFWarning, dpf_warning, sendEngineControl)
DEFINE_BSETTER(setBatteryWarning, battery_warning, sendLights)
DEFINE_BSETTER(setDoorOpenWarning, door_open_warning, sendLights)
DEFINE_BSETTER(setTrunkOpenWarning, trunk_open_warning, sendLights)
DEFINE_BSETTER(setBacklight, backlight, sendLights)
DEFINE_BSETTER(setClutchControl, clutch_control, sendLights)
DEFINE_BSETTER(setCheckLamp, check_lamp, sendLights)
DEFINE_BSETTER(setKeyBatteryWarning, key_battery_warning, sendLights)

void setFogLight(bool fog_light)
{
    dashboard.fog_light = fog_light;
    digitalWrite(FOG_LIGHT_INDICATOR_PIN, fog_light ? HIGH : LOW);
    // sendLights();
}

void setHighBeam(bool high_beam)
{
    dashboard.high_beam = high_beam;
    digitalWrite(HIGH_BEAM_INDICATOR_PIN, high_beam ? HIGH : LOW);
    // sendLights();
}

void updateDashboard()
{
    unsigned long ms = millis();
    unsigned long diff = ms - last_can_millis;
    if (diff >= CAN_REFRESH_RATE)
    {
        last_can_millis += diff / CAN_REFRESH_RATE * CAN_REFRESH_RATE;
        sendImmobilizer();
        sendEngineOnAndESP();
        diff = ms - last_can_light_engine_millis;
        if (diff >= CAN_LIGHT_ENGINE_REFRESH_RATE)
        {
            last_can_light_engine_millis += diff / CAN_LIGHT_ENGINE_REFRESH_RATE * CAN_LIGHT_ENGINE_REFRESH_RATE;
            sendLights();
            sendEngineControl();
        }
        sendSpeedAndDriveMode();
        sendRPM();
        sendSeatBeltWarning();
    }
}

void unsetAll()
{
    dashboard.speed = 0;
    dashboard.rpm = 0;
    dashboard.backlight = false;
    dashboard.turning_lights = 0;
    dashboard.abs = false;
    dashboard.offroad = false;
    setHandbrake(false);
    dashboard.low_tire_pressure = false;
    dashboard.door_open_warning = false;
    dashboard.clutch_control = false;
    dashboard.check_lamp = false;
    dashboard.trunk_open_warning = false;
    dashboard.battery_warning = false;
    dashboard.key_battery_warning = false;
    setFogLight(false);
    setHighBeam(false);
    dashboard.seat_belt_warning = false;
    dashboard.preheating = false;
    dashboard.high_water_temp = false;
    dashboard.dpf_warning = false;
}

void setAll()
{
    dashboard.speed = 100;
    dashboard.rpm = 3000;
    dashboard.backlight = true;
    dashboard.turning_lights = 3;
    dashboard.abs = true;
    dashboard.offroad = true;
    setHandbrake(true);
    dashboard.low_tire_pressure = true;
    dashboard.door_open_warning = true;
    dashboard.clutch_control = true;
    dashboard.check_lamp = true;
    dashboard.trunk_open_warning = true;
    dashboard.battery_warning = true;
    dashboard.key_battery_warning = true;
    setFogLight(true);
    setHighBeam(true);
    dashboard.seat_belt_warning = true;
    dashboard.preheating = true;
    dashboard.high_water_temp = true;
    dashboard.dpf_warning = true;
}

void loop()
{
    updateDashboard();
    if (Serial.available() > 0)
    {
        int i = Serial.parseInt();
        if (i)
        {
            setAll();
        }
        else
        {
            unsetAll();
        }
    }
}
