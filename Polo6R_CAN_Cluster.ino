#include <SPI.h>
#include <avr/io.h>
#include <mcp_can.h>

#define SPI_CS_PIN 10
#define CAN_ID_MODE MCP_ANY
#define CAN_SPEED CAN_500KBPS
#define CAN_CLOCK MCP_8MHZ

#define PARKING_BRAKE_CONTROL_PIN 9
#define FOG_LIGHT_INDICATOR_PIN 8
#define HIGH_BEAM_INDICATOR_PIN 7

#define MAX_RPM 6000U
#define MAX_KMH 240U

#define MSG_VALUE_SEPARATOR ':'
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
    DataCorePlugin.ExternalScript.BlinkingGearUP -> same as gear but changes to
   UP when upshift needed DataCorePlugin.GameData.CarSettings_RPMRedLineReached
   -> 0/1 DataCorePlugin.GameData.CarSettings_RPMShiftLight1 -> 0.0-1.0
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
    unsigned short speed = 0; // speed [km/h]
    unsigned short rpm = 0;   // revs [rpm]
    bool backlight = true;    // dashboard backlight (on/off)
    bool clutch_control =
        false; // display clutch message on dashboard display (on/off)
    bool check_lamp =
        false; // display 'check lamp' message on dashboard display (on/off)
    bool key_battery_warning =
        false;                         // display 'key battery low' message on dashboard display (on/off). probably works only at start
    bool seat_belt_warning = false;    // seat belt warning lamp (on/off)
    bool door_open_warning = false;    // open door warning lamp (on/off)
    bool trunk_open_warning = false;   // open trunk warning lamp (on/off)
    bool parking_brake = false;        // parking brake lamp (on/off)
    bool high_water_temp = false;      // high water temperature lamp (on/off) AVOID IT PLEASE, IT'S VERY LOUD
    bool dpf_warning = false;          // DPF warning lamp (on/off)
    bool preheating = false;           // diesel preheating lamp (on/off)
    bool battery_warning = false;      // battery warning lamp (on/off)
    bool fog_light = false;            // fog light lamp (on/off)
    bool high_beam = false;            // high beam lamp (on/off)
    bool turn_left = false;            // left turn indicator (on/off)
    bool turn_right = false;           // right turn indicator (on/off)
    bool abs = false;                  // ABS lamp (on/off)
    bool traction_control = false;     // traction control lamp (on/off)
    bool handbrake = false;            // handbrake lamp (on/off)
    bool low_tire_pressure = false;    // low tire pressure lamp (on/off)
    bool warning_sound = false;        // warning ding sound, probably open door or unfastened seatbelt (on/off)
    bool airbag_warning = false;       // airbag warning lamp (on/off)
    bool traction_control_off = false; // traction control OFF lamp (on/off)
};

struct CanPacket
{
    short address = 0;
    unsigned char data[8] = {};
    CanPacket() = default;
    CanPacket(short address, byte a, byte b, byte c, byte d, byte e, byte f,
              byte g, byte h)
        : address(address)
    {
        data[0] = a;
        data[1] = b;
        data[2] = c;
        data[3] = d;
        data[4] = e;
        data[5] = f;
        data[6] = g;
        data[7] = h;
    }

    void print()
    {
        Serial.print(address, 16);
        for (unsigned char i = 0; i < 8; ++i)
        {
            Serial.print(';');
            Serial.print(data[i], 16);
        }
    }
};

unsigned long micros_timer_100hz = 0, micros_timer_50hz = 0, micros_timer_10hz = 0, micros_timer_5hz = 0;

namespace Packets
{
    CanPacket immobilizer;
    CanPacket lights;
    CanPacket engine_control;
    CanPacket airbag; // airbag and seat belt info
    CanPacket esp;    // engine and ESP
    CanPacket motor_speed;
    CanPacket rpm;
    CanPacket drive_mode;
    CanPacket abs1;
    CanPacket abs2;
    CanPacket traction;
    CanPacket test_packet;
}

const size_t SERIAL_BUFF_SIZE = 64;
char serial_buffer[SERIAL_BUFF_SIZE];

MCP_CAN can(SPI_CS_PIN);
DashboardSettings dashboard;

void canSend(CanPacket &packet)
{
    can.sendMsgBuf(packet.address, 0, 8, packet.data);
}

void preparePackets()
{
    // for frequencies see https://christian-rossow.de/publications/vatican-ches2016.pdf
    // possibly correct?

    Packets::immobilizer = CanPacket(0x3D0, 0, 0x80, 0, 0, 0, 0, 0, 0); // unknown, possibly 100ms / 10Hz
    Packets::lights = CanPacket(0x470, 0, 0, 0, 0, 0, 0, 0, 0);         // unknown, possibly 100ms / 10Hz
    Packets::engine_control = CanPacket(0x480, 0, 0, 0, 0, 0, 0, 0, 0); // unknown, possibly 100ms / 10Hz
    Packets::airbag = CanPacket(0x050, 0, 0x80, 0, 0, 0, 0, 0, 0);      // 20ms / 50Hz
    Packets::esp = CanPacket(0xDA0, 0xFF, 0, 0, 0, 0, 0, 0, 0xAD);      // unknown, possibly 100ms / 10Hz
    // motor speed? doesn't affect the dashboard at all
    Packets::motor_speed = CanPacket(0x320, 0x04, 0, 0x40, 0, 0, 0, 0, 0);  // 20ms / 50Hz
    Packets::rpm = CanPacket(0x280, 0x49, 0x0E, 0, 0, 0x0E, 0, 0x1B, 0x0E); // 20ms / 50Hz
    // speed, drivemode, potentially mileage
    Packets::drive_mode = CanPacket(0x5A0, 0xFF, 0, 0, 0, 0, 0, 0, 0xAD); // 100ms / 10Hz
    // ABS1: has to be sent to apply the speed?
    Packets::abs1 = CanPacket(0x1A0, 0, 0, 0, 0, 0, 0, 0, 0); // 10ms / 100Hz
    // ABS2: doesn't affect the dashboard?
    Packets::abs2 = CanPacket(0x4A0, 0, 0, 0, 0, 0, 0, 0, 0); // 10ms / 100Hz
    Packets::traction = CanPacket(0x2A0, 0, 0, 0, 0, 0, 0, 0, 0);

    Packets::test_packet = CanPacket(0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);

    resetEverything();
}

void sendPackets(bool hz100, bool hz50, bool hz10, bool hz5)
{
    if (hz100)
    {
        canSend(Packets::immobilizer);
        canSend(Packets::engine_control);
        canSend(Packets::rpm);
        canSend(Packets::airbag);
        canSend(Packets::esp);
        canSend(Packets::abs1);
        canSend(Packets::abs2);
        canSend(Packets::lights);
        canSend(Packets::traction);
        if (Packets::test_packet.address != 0)
        {
            canSend(Packets::test_packet);
        }
    }
    if (hz50)
    {
        canSend(Packets::motor_speed);
    }
    if (hz10)
    {
        canSend(Packets::drive_mode);
    }
}

#define SET_BIT(byte_value, state, bitmask) \
    if (state)                              \
    {                                       \
        byte_value |= bitmask;              \
    }                                       \
    else                                    \
    {                                       \
        byte_value &= ~bitmask;             \
    }

#define BIT_SETTER(setter_name, variable_name, packet, byte_index, bitmask) \
    inline void setter_name(bool on = dashboard.variable_name)              \
    {                                                                       \
        dashboard.variable_name = on;                                       \
        SET_BIT(Packets::packet.data[byte_index], on, bitmask);             \
    }

#define DUAL_BIT_SETTER(setter_name, variable_name, packet1, byte_index1, bitmask1, packet2, byte_index2, bitmask2) \
    inline void setter_name(bool on = dashboard.variable_name)                                                      \
    {                                                                                                               \
        dashboard.variable_name = on;                                                                               \
        SET_BIT(Packets::packet1.data[byte_index1], on, bitmask1);                                                  \
        SET_BIT(Packets::packet2.data[byte_index2], on, bitmask2);                                                  \
    }

#define BIT_PIN_SETTER(setter_name, variable_name, packet, byte_index, bitmask, pin) \
    inline void setter_name(bool on = dashboard.variable_name)                       \
    {                                                                                \
        dashboard.variable_name = on;                                                \
        SET_BIT(Packets::packet.data[byte_index], on, bitmask);                      \
        digitalWrite(pin, on ? HIGH : LOW);                                          \
    }
#define PIN_SETTER(setter_name, variable_name, pin)            \
    inline void setter_name(bool on = dashboard.variable_name) \
    {                                                          \
        dashboard.variable_name = on;                          \
        digitalWrite(pin, on ? HIGH : LOW);                    \
    }

#define INVERTED_PIN_SETTER(setter_name, variable_name, pin)   \
    inline void setter_name(bool on = dashboard.variable_name) \
    {                                                          \
        dashboard.variable_name = on;                          \
        digitalWrite(pin, on ? LOW : HIGH);                    \
    }

inline void setSpeed(unsigned short speed = dashboard.speed)
{
    dashboard.speed = speed;
    const int speed_prescaled =
        static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.007f);
    const byte speed_low = speed_prescaled & 0xFF,
               speed_high = (speed_prescaled >> 8) & 0xFF;

    const int abs_speed_prescaled =
        static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.01f);
    const byte abs_speed_low = abs_speed_prescaled & 0xFF,
               abs_speed_high = (abs_speed_prescaled >> 8) & 0xFF;
    const byte abs_speed15_low = (abs_speed_prescaled << 1) & 0xFF,
               abs_speed15_high = (abs_speed_prescaled >> 7) & 0xFF;

    auto &esp = Packets::esp.data;
    esp[1] = speed_low;
    esp[2] = speed_high;

    // esp[5] = 0x00; trip distance
    // esp[6] = 0x00; to be continued

    auto &motor_speed = Packets::motor_speed.data;
    motor_speed[3] = abs_speed_low;
    motor_speed[4] = abs_speed_high;
    motor_speed[5] = abs_speed15_low;
    motor_speed[6] = abs_speed15_high;

    auto &drive = Packets::drive_mode.data;
    drive[1] = speed_low;
    drive[2] = speed_high;

    auto &abs1 = Packets::abs1.data;
    abs1[1] = 0xA0;
    if (dashboard.abs)
    {
        abs1[1] |= 0x01;
    }
    abs1[2] = abs_speed15_low;
    abs1[3] = abs_speed15_high;

    auto &abs2 = Packets::abs2.data;
    abs2[0] = abs_speed15_low;
    abs2[1] = abs_speed15_high;
    abs2[2] = abs_speed15_low;
    abs2[3] = abs_speed15_high;
    abs2[4] = abs_speed15_low;
    abs2[5] = abs_speed15_high;
    abs2[6] = abs_speed15_low;
    abs2[7] = abs_speed15_high;
}

inline void setRPM(unsigned short rpm = dashboard.rpm)
{
    dashboard.rpm = rpm;
    const unsigned short rpm_prescaled = rpm * 4;
    Packets::rpm.data[2] = rpm_prescaled & 0xFF;
    Packets::rpm.data[3] = (rpm_prescaled >> 8) & 0xFF;
}

inline void setTractionControl(bool on = dashboard.traction_control)
{
    dashboard.traction_control = on;
    SET_BIT(Packets::esp.data[3], on, 0x02)
    SET_BIT(Packets::drive_mode.data[3], on, 0x02)
    if (dashboard.traction_control_off)
    {
        SET_BIT(Packets::traction.data[3], on, 0x8)
    }
    else
    {
        SET_BIT(Packets::traction.data[3], on, 0xC)
    }
}

inline void setTractionControlOff(bool on = dashboard.traction_control_off)
{
    dashboard.traction_control_off = on;
    if (!dashboard.traction_control)
    {
        SET_BIT(Packets::traction.data[3], on, 0xF4)
    }
    else
    {
        SET_BIT(Packets::traction.data[3], on, 0xF0)
    }
}

BIT_SETTER(setBacklight, backlight, lights, 2, 0x01)
BIT_SETTER(setClutchControl, clutch_control, lights, 4, 0x01)
BIT_SETTER(setCheckLamp, check_lamp, lights, 4, 0x10)
BIT_SETTER(setKeyBatteryWarning, key_battery_warning, lights, 5, 0x80)
BIT_SETTER(setSeatBeltWarning, seat_belt_warning, airbag, 2, 0x04)
BIT_SETTER(setDoorOpenWarning, door_open_warning, lights, 1, 0x01)
BIT_SETTER(setTrunkOpenWarning, trunk_open_warning, lights, 1, 0x20)
INVERTED_PIN_SETTER(setParkingBrake, parking_brake, PARKING_BRAKE_CONTROL_PIN)
BIT_SETTER(setHighWaterTemp, high_water_temp, engine_control, 1, 0x01)
BIT_SETTER(setDPFWarning, dpf_warning, engine_control, 5, 0x02)
BIT_SETTER(setPreheating, preheating, engine_control, 1, 0x02)
BIT_SETTER(setBatteryWarning, battery_warning, lights, 0, 0x80)
BIT_PIN_SETTER(setFogLight, fog_light, lights, 7, 0x20, FOG_LIGHT_INDICATOR_PIN)
BIT_PIN_SETTER(setHighBeam, high_beam, lights, 7, 0x40, HIGH_BEAM_INDICATOR_PIN)
BIT_SETTER(setTurnLeft, turn_left, lights, 0, 0x01)
BIT_SETTER(setTurnRight, turn_right, lights, 0, 0x02)
DUAL_BIT_SETTER(setABS, abs, esp, 3, 0x01, drive_mode, 3, 0x01)
DUAL_BIT_SETTER(setHandbrake, handbrake, esp, 3, 0x04, drive_mode, 3, 0x04)
DUAL_BIT_SETTER(setLowTirePressure, low_tire_pressure, esp, 3, 0x08, drive_mode, 3, 0x08)
BIT_SETTER(setWarningSound, warning_sound, esp, 4, 0x30)
BIT_SETTER(setAirbagWarning, airbag_warning, airbag, 1, 0x01)

inline void resetEverything()
{
    setSpeed();
    setRPM();
    setBacklight();
    setClutchControl();
    setCheckLamp();
    setKeyBatteryWarning();
    setSeatBeltWarning();
    setDoorOpenWarning();
    setTrunkOpenWarning();
    setParkingBrake();
    setHighWaterTemp();
    setDPFWarning();
    setPreheating();
    setBatteryWarning();
    setFogLight();
    setHighBeam();
    setTurnLeft();
    setTurnRight();
    setABS();
    setTractionControl();
    setHandbrake();
    setLowTirePressure();
    setWarningSound();
    setAirbagWarning();
    setTractionControlOff();
}

inline int str2int(const char *str, int len)
{
    int ret = 0;
    for (int i = 0; i < len; ++i)
    {
        ret = ret * 10 + (str[i] - '0');
    }
    return ret;
}

inline int getSeparatorIndex(const char *str, int len)
{
    for (int i = 0; i < len; ++i)
    {
        if (str[i] == MSG_VALUE_SEPARATOR)
        {
            return i;
        }
    }
    return -1;
}

bool processSerialCommand()
{
    if (!Serial.available())
    {
        return false;
    }
    if (Serial.peek() == '\n')
    {
        Serial.read();
        return false;
    }
    int len = Serial.readBytesUntil(MSG_DELIMITER, serial_buffer, SERIAL_BUFF_SIZE);
    if (len < 2 && !(len == 1 && serial_buffer[0] == 'x'))
    {
        return true;
    }
    const char header = serial_buffer[0];
    char *buffer = serial_buffer + 1;
    --len;
    switch (header)
    {
    case 'x': // debugging: set test packet address
    {
        if (len == 0)
        {
            ++Packets::test_packet.address;
        }
        else
        {
            buffer[len] = '\0';
            Packets::test_packet.address = strtol(buffer, nullptr, 16);
        }
        Packets::test_packet.print();
        Serial.println();
        break;
    }
    case 'y': // debugging: set test packet value
    {
        buffer[len] = '\0';
        for (unsigned char i = 0; i < 8; ++i)
        {
            Packets::test_packet.data[i] = strtol(buffer, &buffer, 16);
        }
        Packets::test_packet.print();
        Serial.println();
        break;
    }
    case 'A': // speed[:max_speed] (int)
    {
        const int separator_index = getSeparatorIndex(buffer, len);
        if (separator_index == 0)
        {
            break;
        }
        const bool no_max_speed =
            separator_index == (len - 1) || separator_index == -1;
        int speed = str2int(buffer,
                            (separator_index == -1) ? len : separator_index);
#ifdef RESCALE_KMH
        if (!no_max_speed)
        {
            const int max_speed = str2int(buffer + separator_index + 1,
                                          len - separator_index - 1);
            if (max_speed != 0)
            {
                speed = map(speed, 0, max_speed, 0, MAX_KMH);
            }
        }
#endif
        if (speed > MAX_KMH)
        {
            speed = MAX_KMH;
        }
        else if (speed < 0)
        {
            speed = 0;
        }
        setSpeed(speed);
        break;
    }
    case 'B': // rpm[:max_rpm] (int)
    {
        const int separator_index = getSeparatorIndex(buffer, len);
        if (separator_index == 0)
        {
            break;
        }
        const bool no_max_rpm =
            separator_index == (len - 1) || separator_index == -1;
        int rpm = str2int(buffer,
                          (separator_index == -1) ? len : separator_index);
#ifdef RESCALE_RPM
        if (!no_max_rpm)
        {
            const int max_rpm = str2int(buffer + separator_index + 1,
                                        len - separator_index - 1);
            if (max_rpm != 0)
            {
                rpm = map(rpm, 0, max_rpm, 0, MAX_RPM);
            }
        }
#endif
        if (rpm > MAX_RPM)
        {
            rpm = MAX_RPM;
        }
        else if (rpm < 0)
        {
            rpm = 0;
        }
        setRPM(rpm);
        break;
    }
    case 'C': // Backlight (bool)
        setBacklight(*buffer == '1');
        break;
    case 'D': // Clutch control warning (bool)
        setClutchControl(*buffer == '1');
        break;
    case 'E': // Check lamp message (bool)
        setCheckLamp(*buffer == '1');
        break;
    case 'F': // Key battery warning (bool)
        setKeyBatteryWarning(*buffer == '1');
        break;
    case 'G': // Seat belt warning (bool)
        setSeatBeltWarning(*buffer == '1');
        break;
    case 'H': // Door open warning (bool)
        setDoorOpenWarning(*buffer == '1');
        break;
    case 'I': // Trunk open warning (bool)
        setTrunkOpenWarning(*buffer == '1');
        break;
    case 'J': // Parking brake (bool)
        setParkingBrake(*buffer == '1');
        break;
    case 'K': // High water temperature (bool)
        setHighWaterTemp(*buffer == '1');
        break;
    case 'L': // DPF warning (bool)
        setDPFWarning(*buffer == '1');
        break;
    case 'M': // Preheating (bool)
        setPreheating(*buffer == '1');
        break;
    case 'N': // Battery warning (bool)
        setBatteryWarning(*buffer == '1');
        break;
    case 'O': // Fog light (bool)
        setFogLight(*buffer == '1');
        break;
    case 'P': // High beam (bool)
        setHighBeam(*buffer == '1');
        break;
    case 'Q': // Left turn indicator (bool)
        setTurnLeft(*buffer == '1');
        break;
    case 'R': // Right turn indicator (bool)
        setTurnRight(*buffer == '1');
        break;
    case 'S': // ABS (bool)
        setABS(*buffer == '1');
        break;
    case 'T': // Traction control (bool)
        setTractionControl(*buffer == '1');
        break;
    case 'U': // Handbrake (bool)
        setHandbrake(*buffer == '1');
        break;
    case 'V': // Low tire pressure (bool)
        setLowTirePressure(*buffer == '1');
        break;
    case 'X': // Warning sound (bool)
        setWarningSound(*buffer == '1');
        break;
    case 'Y': // Airbag warning (bool)
        setAirbagWarning(*buffer == '1');
        break;
    case 'Z': // Traction control OFF (bool)
        setTractionControlOff(*buffer == '1');
        break;
    default:
        break;
    }
    return true;
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
    preparePackets();
    micros_timer_5hz = micros_timer_10hz = micros_timer_50hz = micros_timer_100hz = micros();
}

void loop()
{
    const unsigned long us = micros();
    bool hz100 = false, hz50 = false, hz10 = false, hz5 = false;
    if ((us - micros_timer_100hz) >= 10000) // 10ms / 100Hz
    {
        hz100 = true;
        micros_timer_100hz += 10000;
    }
    if ((us - micros_timer_50hz) >= 20000) // 20ms / 50Hz
    {
        hz50 = true;
        micros_timer_50hz += 20000;
    }
    if ((us - micros_timer_10hz) >= 100000) // 100ms / 10Hz
    {
        hz10 = true;
        micros_timer_10hz += 100000;
    }
    if ((us - micros_timer_5hz) >= 200000) // 200ms / 5Hz
    {
        hz5 = true;
        micros_timer_5hz += 200000;
    }
    sendPackets(hz100, hz50, hz10, hz5);
    processSerialCommand();
}
