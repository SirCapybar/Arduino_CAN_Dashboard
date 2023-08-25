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
#define MIN_OIL_TEMP 50U
#define MAX_OIL_TEMP 194U

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

/*
Packet discovery notes:
- "check lamp" symbol can also be triggered with packet 0x394
- bit 0x04 in the shift lock byte produces overheat warning (loud)
- byte 1 and 2 from 0x48A seem to also do something to the RPM
- packet 0x550 draws rear passenger seatbelt status (disappears after a while). Byte 3 contains a bitmask for the 3 seats, probably 2 bits per seat
- packet 0x58C can also trigger shift lock lamp (byte 7). It also contains brake warning (byte 0)
- 0x40 in byte 3 of 0x5D0 produces "TRA" notification, whatever it means
- packet 0x850 also triggers airbag/seatbelt warnings
*/

struct DashboardSettings
{
    unsigned short speed = 0;              // speed [km/h]
    unsigned short rpm = 0;                // revs [rpm]
    unsigned char oil_temp = MIN_OIL_TEMP; // oil temperature [°C]
    size_t water_temp_index = 0;           // water temperature value index (see arrays below)
    bool backlight = true;                 // dashboard backlight (on/off)
    bool clutch_control =
        true; // display clutch message on dashboard display (on/off)
    bool check_lamp =
        false; // display 'check lamp' warning lamp (on/off)
    bool key_battery_warning =
        false;                         // display 'key battery low' message on dashboard display (on/off). probably works only at start
    bool seat_belt_warning = false;    // seat belt warning lamp (on/off)
    bool door_open_warning = false;    // open door warning lamp (on/off)
    bool trunk_open_warning = false;   // open trunk warning lamp (on/off)
    bool parking_brake = false;        // parking brake lamp (on/off)
    bool cruise_control = false;       // cruise control lamp (on/off)
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
    bool ding_sound = false;           // Another warning ding sound, open door/unfastened seatbelt I suppose (on/off)
    bool shift_lock = false;           // Shift lock lamp (on/off)
};

struct CanPacket
{
    unsigned long address = 0;
    unsigned char data[8] = {};
    unsigned char length = 8;
    CanPacket() = default;
    CanPacket(unsigned long address, byte a, byte b, byte c, byte d, byte e, byte f,
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

    CanPacket &operator=(const CanPacket &other)
    {
        if (this == &other)
        {
            return *this;
        }
        address = other.address;
        length = other.length;
        for (unsigned char i = 0; i < length; ++i)
        {
            data[i] = other.data[i];
        }
        return *this;
    }

    unsigned char &operator[](int index)
    {
        return data[index];
    }

    const unsigned char &operator[](int index) const
    {
        return data[index];
    }

    void copyDataTo(CanPacket &packet) const
    {
        packet.length = length;
        for (unsigned char i = 0; i < length; ++i)
        {
            packet[i] = data[i];
        }
    }

    void print() const
    {
        Serial.print(address, 16);
        for (unsigned char i = 0; i < length; ++i)
        {
            Serial.print(' ');
            Serial.print(data[i], 16);
        }
    }

    void print(bool is_input) const
    {
        Serial.print(is_input ? "< " : "> ");
        Serial.print(address, 16);
        for (unsigned char i = 0; i < length; ++i)
        {
            Serial.print(' ');
            Serial.print(data[i], 16);
        }
        Serial.println();
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
    CanPacket water_temp; // water temperature and cruise control indicator
    CanPacket oil_temp;   // oil temperature
    CanPacket drive_mode;
    CanPacket abs1;
    CanPacket abs2;
    CanPacket traction;
    CanPacket ding;
    CanPacket shift;
    CanPacket test_packet;
}

bool is_speed_resetting = false;
const size_t SERIAL_BUFF_SIZE = 64, INCOMING_CAN_BUFFER_SIZE = 16, WATER_TEMPS_SIZE = 31;
char serial_buffer[SERIAL_BUFF_SIZE];
unsigned char incoming_can_buffer[INCOMING_CAN_BUFFER_SIZE];
const signed char WATER_TEMPS[WATER_TEMPS_SIZE] = {-45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 10, 20, 30, 40, 50, 55, 60, 70, 80, 90, 100, 105, 110, 115, 120, 125, 126, 127, 128, 129, 130};
const unsigned char WATER_TEMP_BYTES[WATER_TEMPS_SIZE] = {0x01, 0x08, 0x0E, 0x15, 0x1C, 0x22, 0x29, 0x30, 0x36, 0x3D, 0x4A, 0x58, 0x66, 0x72, 0x80, 0x85, 0x89, 0x92, 0x9A, 0xA2, 0xD4, 0xD7, 0xDB, 0xDE, 0xE1, 0xE6, 0xE8, 0xE9, 0xEA, 0xEC, 0xED};

MCP_CAN can(SPI_CS_PIN);
DashboardSettings dashboard;

void canSend(CanPacket &packet)
{
    can.sendMsgBuf(packet.address, 0, packet.length, packet.data);
}

void preparePackets()
{
    // for frequencies see https://christian-rossow.de/publications/vatican-ches2016.pdf
    // possibly correct?

    Packets::immobilizer = CanPacket(0x3D0, 0, 0x80, 0, 0, 0, 0, 0, 0); // unknown, possibly 100ms / 10Hz
    Packets::lights = CanPacket(0x470, 0, 0, 0, 0, 0, 0, 0, 0);         // unknown, possibly 100ms / 10Hz
    Packets::engine_control = CanPacket(0x480, 0, 0, 0, 0, 0, 0, 0, 0); // unknown, possibly 100ms / 10Hz
    Packets::airbag = CanPacket(0x050, 0, 0x80, 0, 0, 0, 0, 0, 0);      // 20ms / 50Hz
    Packets::esp = CanPacket(0xDA0, 0, 0, 0, 0, 0, 0, 0, 0);            // unknown, possibly 100ms / 10Hz
    // motor speed? doesn't affect the dashboard at all
    Packets::motor_speed = CanPacket(0x320, 0x06, 0, 0, 0, 0, 0, 0, 0x80);  // 20ms / 50Hz
    Packets::rpm = CanPacket(0x280, 0x49, 0x0E, 0, 0, 0x0E, 0, 0x1B, 0x0E); // 20ms / 50Hz
    Packets::water_temp = CanPacket(0x288, 0, 0, 0, 0, 0, 0, 0, 0);         // unknown, possibly 100ms / 10Hz
    Packets::oil_temp = CanPacket(0x588, 0, 0, 0, 0, 0, 0, 0, 0);
    // speed, drivemode, potentially mileage
    Packets::drive_mode = CanPacket(0x5A0, 0xFF, 0, 0, 0, 0, 0, 0, 0xAD); // 100ms / 10Hz
    // ABS1: has to be sent to apply the speed?
    Packets::abs1 = CanPacket(0x1A0, 0, 0xA0, 0, 0, 0, 0, 0, 0); // 10ms / 100Hz
    // ABS2: doesn't affect the dashboard?
    Packets::abs2 = CanPacket(0x4A0, 0, 0, 0, 0, 0, 0, 0, 0); // 10ms / 100Hz
    Packets::traction = CanPacket(0x2A0, 0, 0, 0, 0, 0, 0, 0, 0);
    Packets::ding = CanPacket(0x2C0, 0, 0, 0, 0, 0, 0, 0, 0);
    Packets::shift = CanPacket(0x540, 0, 0, 0, 0, 0, 0, 0, 0);

    Packets::test_packet = CanPacket(0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);

    resetEverything();
}

inline size_t getWaterTempIndex(int water_temp_value)
{
    if (water_temp_value <= static_cast<int>(WATER_TEMPS[0]))
    {
        return 0;
    }
    if (water_temp_value >= static_cast<int>(WATER_TEMPS[WATER_TEMPS_SIZE - 1]))
    {
        return WATER_TEMPS_SIZE - 1;
    }
    signed char water_temp = static_cast<signed char>(water_temp_value);
    for (size_t i = 0; i < WATER_TEMPS_SIZE - 1; ++i)
    {
        signed char curr = WATER_TEMPS[i], next = WATER_TEMPS[i + 1];
        if (water_temp > next)
        {
            continue;
        }
        if (water_temp == curr)
        {
            return i;
        }
        if (water_temp == next)
        {
            return i + 1;
        }
        if ((water_temp - curr) <= (next - water_temp))
        {
            return i;
        }
        return i + 1;
    }
    return 0; // for safety
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
        SET_BIT(Packets::packet[byte_index], on, bitmask);                  \
    }

#define DUAL_BIT_SETTER(setter_name, variable_name, packet1, byte_index1, bitmask1, packet2, byte_index2, bitmask2) \
    inline void setter_name(bool on = dashboard.variable_name)                                                      \
    {                                                                                                               \
        dashboard.variable_name = on;                                                                               \
        SET_BIT(Packets::packet1[byte_index1], on, bitmask1);                                                       \
        SET_BIT(Packets::packet2[byte_index2], on, bitmask2);                                                       \
    }

#define BIT_PIN_SETTER(setter_name, variable_name, packet, byte_index, bitmask, pin) \
    inline void setter_name(bool on = dashboard.variable_name)                       \
    {                                                                                \
        dashboard.variable_name = on;                                                \
        SET_BIT(Packets::packet[byte_index], on, bitmask);                           \
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
    using namespace Packets;
    dashboard.speed = speed;

    if (is_speed_resetting)
    {
        esp[1] = 0;
        esp[2] = 0;
        motor_speed[3] = 0;
        motor_speed[4] = 0;
        motor_speed[5] = 0;
        motor_speed[6] = 0;
        drive_mode[1] = 0;
        drive_mode[2] = 0;
        abs1[2] = 0x01;
        abs1[3] = 0x01;
        abs2[0] = 0x01;
        abs2[1] = 0;
        abs2[2] = 0x01;
        abs2[3] = 0;
        abs2[4] = 0x01;
        abs2[5] = 0;
        abs2[6] = 0x01;
        abs2[7] = 0;
        return;
    }

    const unsigned short speed_prescaled =
        static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.007f);
    const unsigned char speed_low = speed_prescaled & 0xFF,
                        speed_high = (speed_prescaled >> 8) & 0xFF;

    const unsigned short abs_speed_prescaled =
        static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.01f);
    const unsigned char abs_speed_low = abs_speed_prescaled & 0xFF,
                        abs_speed_high = (abs_speed_prescaled >> 8) & 0xFF;
    const unsigned char abs_speed15_low = ((abs_speed_prescaled << 1) & 0xFF) | 0x01,
                        abs_speed15_high = (abs_speed_prescaled >> 7) & 0xFF;

    esp[1] = speed_low;
    esp[2] = speed_high;

    // esp[5] = random() % 256;
    // esp[6] = 0x08;
    // esp[5] = 0x00; trip distance
    // esp[6] = 0x00; to be continued

    motor_speed[3] = abs_speed_low;
    motor_speed[4] = abs_speed_high;
    motor_speed[5] = abs_speed15_low;
    motor_speed[6] = abs_speed15_high;

    drive_mode[1] = speed_low;
    drive_mode[2] = speed_high;

    abs1[2] = abs_speed15_low;
    abs1[3] = abs_speed15_high;

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
    Packets::rpm[2] = rpm_prescaled & 0xFF;
    Packets::rpm[3] = (rpm_prescaled >> 8) & 0xFF;
}

inline void setWaterTempIndex(size_t water_temp_index = dashboard.water_temp_index)
{
    if (water_temp_index < 0)
    {
        water_temp_index = 0;
    }
    else if (water_temp_index >= WATER_TEMPS_SIZE)
    {
        water_temp_index = WATER_TEMPS_SIZE - 1;
    }
    dashboard.water_temp_index = water_temp_index;
    Packets::water_temp[1] = WATER_TEMP_BYTES[water_temp_index];
}

inline void setOilTemp(unsigned char oil_temp)
{
    dashboard.oil_temp = oil_temp;
    Packets::oil_temp[7] = oil_temp + 60;
}

inline void setTractionControl(bool on = dashboard.traction_control)
{
    dashboard.traction_control = on;
    SET_BIT(Packets::esp[3], on, 0x02)
    SET_BIT(Packets::drive_mode[3], on, 0x02)
    if (dashboard.traction_control_off)
    {
        SET_BIT(Packets::traction[3], on, 0x8)
    }
    else
    {
        SET_BIT(Packets::traction[3], on, 0xC)
    }
}

inline void setTractionControlOff(bool on = dashboard.traction_control_off)
{
    dashboard.traction_control_off = on;
    if (!dashboard.traction_control)
    {
        SET_BIT(Packets::traction[3], on, 0xF4)
    }
    else
    {
        SET_BIT(Packets::traction[3], on, 0xF0)
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
BIT_SETTER(setCruiseControl, cruise_control, water_temp, 2, 0x80)
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
BIT_SETTER(setDingSound, ding_sound, ding, 7, 0x04)
BIT_SETTER(setShiftLock, shift_lock, shift, 6, 0x10)

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
    setDingSound();
    setShiftLock();
}

void sendPackets(bool hz100, bool hz50, bool hz10, bool hz5)
{
    using namespace Packets;
    if (hz100)
    {
        canSend(immobilizer);
        canSend(engine_control);
        canSend(rpm);
        canSend(water_temp);
        canSend(oil_temp);
        canSend(airbag);
        canSend(esp);
        canSend(abs1);
        canSend(abs2);
        canSend(lights);
        if (test_packet.address != 0)
        {
            canSend(test_packet);
        }
    }
    if (hz50)
    {
        canSend(motor_speed);
        canSend(ding);
        canSend(traction);
        canSend(shift);
    }
    if (hz10)
    {
        canSend(drive_mode);
    }
}

void handleIncomingPacket(const CanPacket &packet)
{
    switch (packet.address)
    {
    case 0x420:
        break;
    case 0x51A:
        break;
    case 0x520:
        break;
    case 0x52A:
        break;
    case 0x5D2:
        break;
    case 0x5F3:
        break;
    case 0x60E:
        break;
    case 0x621:
        break;
    case 0x62E:
        break;
    case 0x727:
        break;
    case 0x320:
    {
        const bool reset_speed = packet[3] == 0xFF && packet[4] == 0xFF && packet[6] == 0xFF;
        const bool should_update_speed = reset_speed != is_speed_resetting;
        is_speed_resetting = reset_speed;
        if (should_update_speed)
        {
            setSpeed();
        }
        break;
    }
    default:
        packet.print(true);
        break;
    }
}

void receivePackets()
{
    if (can.checkReceive() == CAN_MSGAVAIL)
    {
        CanPacket packet;
        if (can.readMsgBuf(&packet.address, &packet.length, packet.data) == CAN_NOMSG)
        {
            return;
        }
        handleIncomingPacket(packet);
    }
}

inline int str2int(const char *str, int len)
{
    bool negate = str[0] == '-';
    int ret = 0;
    for (int i = negate ? 1 : 0; i < len; ++i)
    {
        ret = ret * 10 + (str[i] - '0');
    }
    return negate ? -ret : ret;
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
            Packets::test_packet[i] = strtol(buffer, &buffer, 16);
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
    case 'a': // Ding sound (bool)
        setDingSound(*buffer == '1');
        break;
    case 'b': // Shift lock (bool)
        setShiftLock(*buffer == '1');
        break;
    case 'c': // Water temperature (int)
        const int separator_index = getSeparatorIndex(buffer, len);
        if (separator_index == 0)
        {
            break;
        }
        int water_temp = str2int(buffer,
                                 (separator_index == -1) ? len : separator_index);
        size_t water_temp_index = getWaterTempIndex(water_temp);
        setWaterTempIndex(water_temp_index);
        break;
    case 'd': // Cruise control (bool)
        setCruiseControl(*buffer == '1');
        break;
    case 'e': // Oil temperature (int)
        const int separator_index = getSeparatorIndex(buffer, len);
        if (separator_index == 0)
        {
            break;
        }
        int oil_temp = str2int(buffer,
                               (separator_index == -1) ? len : separator_index);
        if (oil_temp < MIN_OIL_TEMP)
        {
            oil_temp = MIN_OIL_TEMP;
        }
        else if (oil_temp > MAX_OIL_TEMP)
        {
            oil_temp = MAX_OIL_TEMP;
        }
        setOilTemp(oil_temp);
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
    receivePackets();
    processSerialCommand();
}
