#include <mcp_can.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define SPI_CS_PIN 10
#define CAN_ID_MODE MCP_ANY
#define CAN_SPEED CAN_500KBPS
#define CAN_CLOCK MCP_8MHZ

#define HANDBRAKE_CONTROL_PIN 9
#define FOG_LIGHT_INDICATOR_PIN 8
#define HIGH_BEAM_INDICATOR_PIN 7

#define CAN_50HZ_PRESCALER 2  // 2 cycles of 100Hz interrupts
#define CAN_20HZ_PRESCALER 5  // 5 cycles of 100Hz interrupts
#define CAN_10HZ_PRESCALER 10 // 10 cycles of 100Hz interrupts

#define MAX_RPM 6000U
#define MAX_KMH 240U

#define RESCALE_RPM

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
    bool backlight = false;           // dashboard backlight (off/on)
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
byte can_10hz_counter = 0;

void canSend(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h)
{
    unsigned char data[8] = {a, b, c, d, e, f, g, h};
    can.sendMsgBuf(address, 0, 8, data);
}

void setup()
{
    pinMode(HANDBRAKE_CONTROL_PIN, OUTPUT);
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

    // configure 16-bit timer
    cli();                                     // Disable interrupts
    TCNT1 = 0;                                 // Initialize timer value with 0
    TCCR1A = 0;                                // Disable timer output pins, zero WGM11:0 for CTC mode with OCR1A
    TCCR1B = 1 << WGM12 | 1 << CS12;           // Set WGM12 for CTC mode with OCR1A, set CS12:0 to prescaler of 256
    OCR1A = static_cast<unsigned short>(624U); // So that we get frequency of 100Hz=16'000'000/(256*(624+1))
    TIMSK1 = 1 << OCIE1A;                      // Enable timer interrupt A
    sei();                                     // Enable interrupts
}

ISR(TIMER1_COMPA_vect)
{
    bool can_10hz = can_10hz_counter % CAN_10HZ_PRESCALER == 0;
    bool can_20hz = can_10hz_counter % CAN_20HZ_PRESCALER == 0;
    bool can_50hz = can_10hz_counter % CAN_50HZ_PRESCALER == 0;

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

    if (can_50hz)
    {
        // airbag and seat belt info
        byte seat_belt = 0;
        if (dashboard.seat_belt_warning)
        {
            seat_belt = 0x04;
        }
        canSend(0x050, 0, 0x80, seat_belt, 0, 0, 0, 0, 0);
    }

    if (can_20hz)
    {
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
        digitalWrite(HANDBRAKE_CONTROL_PIN, dashboard.handbrake ? LOW : HIGH);

        // ABS1: has to be sent to apply the speed, though it can all be zeros?
        canSend(0x1A0, 0, 0x18, abs_speed15_low, abs_speed15_high, 0xFE, 0xFE, 0, 0x00);

        // ABS2: doesn't affect the dashboard?
        canSend(0x4A0, abs_speed15_low, abs_speed15_high, abs_speed15_low, abs_speed15_high, abs_speed15_low, abs_speed15_high, abs_speed15_low, abs_speed15_high);
    }

    if (++can_10hz_counter == CAN_10HZ_PRESCALER)
    {
        can_10hz_counter = 0;
    }
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
        default:
        {
            String s = Serial.readString();
            switch (c)
            {
            // for simhub
            case 'P':
                dashboard.door_open_warning = s[0] == '1';
                break;
            case 'R':
            {
                int sep = s.indexOf(';');
                int rpm = s.substring(0, sep).toInt();
#ifdef RESCALE_RPM
                int max_rpm = s.substring(sep + 1).toInt();
                rpm = map(rpm, 0, max_rpm, 0, MAX_RPM);
#endif
                dashboard.rpm = rpm > MAX_RPM ? MAX_RPM : rpm;
                break;
            }
            case 'S':
            {
                unsigned short speed = static_cast<unsigned short>(s.toInt());
                dashboard.speed = speed > MAX_KMH ? MAX_KMH : speed;
                break;
            }
            case 'A':
                dashboard.abs = s[0] == '1';
                break;
            case 'H':
            {
                bool handbrake = s[0] == '1';
                bool parking_brake = s[2] == '1';
                dashboard.handbrake == handbrake || parking_brake; //todo: this can be separated probably, there are 2 controls in the dashboard
                break;
            }
            case 'G':
                // todo
                break;
            case 'T':
            {
                bool left = s[0] == '1';
                bool right = s[2] == '1';
                if (left)
                {
                    dashboard.turning_lights = right ? 3 : 1;
                }
                else
                {
                    dashboard.turning_lights = right ? 2 : 0;
                }
                break;
            }
            case 'L':
                dashboard.fog_light = s[0] == '1'; // acts as low beam
                dashboard.high_beam = s[2] == '1';
                break;
            case 'B':
                dashboard.battery_warning = s[0] == '1';
                break;
            case 'W':
                dashboard.high_water_temp = s[0] == '1';
                break;
            case 'D':
                dashboard.backlight = s[0] == '1';
                break;
            }
        }
        break;
        // for debugging only
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
            cli();
            dashboard.speed += 50;
            if (dashboard.speed > MAX_KMH)
            {
                dashboard.speed = 0;
            }
            sei();
            break;
        case 't':
            cli();
            dashboard.rpm += 500;
            if (dashboard.rpm > MAX_RPM)
            {
                dashboard.rpm = 0;
            }
            sei();
            break;
        }
    }
}
