#include <SPI.h>
#include <avr/interrupt.h>
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
    bool backlight = false;   // dashboard backlight (off/on)
    byte turning_lights =
        0;                          // turning lights (0: off, 1: left, 2: right, 3: both)
    bool abs = false;               // ABS lamp (on/off)
    bool offroad = false;           // offroad lamp (on/off)
    bool handbrake = false;         // handbrake lamp (on/off)
    bool parking_brake = false;     // parking brake lamp (on/off)
    bool low_tire_pressure = false; // low tire pressure lamp (on/off)
    bool door_open_warning = false; // open door warning lamp (on/off)
    bool clutch_control =
        false; // display clutch message on dashboard display (on/off)
    bool check_lamp =
        false;                       // display 'check lamp' message on dashboard display (on/off)
    bool trunk_open_warning = false; // open trunk warning lamp (on/off)
    bool battery_warning = false;    // battery warning lamp (on/off)
    bool key_battery_warning =
        false;                      // display 'key battery low' message on dashboard display. probably
                                    // works only at start (on/off)
    bool fog_light = false;         // fog light lamp (on/off)
    bool high_beam = false;         // high beam lamp (on/off)
    bool seat_belt_warning = false; // seat belt warning lamp (on/off)
    bool preheating = false;        // diesel preheating lamp (on/off)
    bool high_water_temp = false;   // high water temperature lamp (on/off) AVOID IT
                                    // PLEASE, BEEPS AWFULLY LOUD
    bool dpf_warning = false;       // DPF warning lamp (on/off)
};

struct CanPacket
{
    short address;
    unsigned char data[8];
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
};

CanPacket buffer1[10], buffer2[10];
CanPacket *write_buffer = buffer1, *send_buffer = buffer2;
bool write_buffer_ready = false, send_buffer_ready = false, buffer_sent = false;

const size_t SERIAL_BUFFER_SIZE = 64;
char serial_buffer[SERIAL_BUFFER_SIZE];

MCP_CAN can(SPI_CS_PIN);
DashboardSettings dashboard;

void swapBuffers()
{
    CanPacket *tmp = send_buffer;
    send_buffer = write_buffer;
    write_buffer = tmp;
}

void canSend(short address, byte a, byte b, byte c, byte d, byte e, byte f,
             byte g, byte h)
{
    unsigned char data[8] = {a, b, c, d, e, f, g, h};
    can.sendMsgBuf(address, 0, 8, data);
}

void canSend(CanPacket &packet)
{
    can.sendMsgBuf(packet.address, 0, 8, packet.data);
}

void preparePackets()
{
    // immobilizer
    write_buffer[0] = CanPacket(0x3D0, 0, 0x80, 0, 0, 0, 0, 0, 0);
    // lights
    write_buffer[1] = CanPacket(0x470, 0, 0, 0, 0, 0, 0, 0, 0);
    // engine control
    write_buffer[2] = CanPacket(0x480, 0, 0, 0, 0, 0, 0, 0, 0);
    // airbag and seat belt info
    write_buffer[3] = CanPacket(0x050, 0, 0x80, 0, 0, 0, 0, 0, 0);
    // engine on and ESP
    write_buffer[4] = CanPacket(0xDA0, 0x01, 0x80, 0, 0, 0, 0, 0, 0);
    // motor speed? doesn't affect the dashboard at all
    write_buffer[5] = CanPacket(0x320, 0x04, 0, 0x40, 0, 0, 0, 0, 0);
    // rpm
    write_buffer[6] = CanPacket(0x280, 0x49, 0x0E, 0, 0, 0, 0, 0, 0);
    // speed, drivemode, potentially mileage
    write_buffer[7] = CanPacket(0x5A0, 0xFF, 0, 0, 0, 0, 0, 0, 0xAD);
    // ABS1: has to be sent to apply the speed, though it can all be zeros?
    write_buffer[8] = CanPacket(0x1A0, 0, 0x18, 0, 0, 0xFE, 0xFE, 0, 0x00);
    // ABS2: doesn't affect the dashboard?
    write_buffer[9] = CanPacket(0x4A0, 0, 0, 0, 0, 0, 0, 0, 0);

    fillPacketBuffer();
}

void fillPacketBuffer()
{
    auto &lights = write_buffer[1].data;
    lights[0] = dashboard.turning_lights & 0x03;
    if (dashboard.battery_warning)
    {
        lights[0] |= 0x80;
    }
    lights[1] = 0;
    if (dashboard.door_open_warning)
    {
        lights[1] |= 0x01;
    }
    if (dashboard.trunk_open_warning)
    {
        lights[1] |= 0x20;
    }
    lights[2] = 0;
    if (dashboard.backlight)
    {
        lights[2] |= 0x01;
    }
    lights[4] = 0;
    if (dashboard.clutch_control)
    {
        lights[4] |= 0x01;
    }
    if (dashboard.check_lamp)
    {
        lights[4] |= 0x10;
    }
    lights[5] = 0;
    if (dashboard.key_battery_warning)
    {
        lights[5] |= 0x80;
    }
    lights[7] = 0;
    if (dashboard.fog_light)
    {
        lights[7] |= 0x20;
    }
    if (dashboard.high_beam)
    {
        lights[7] |= 0x40;
    }

    // engine control
    auto &engine_control = write_buffer[2].data;
    engine_control[1] = 0;
    if (dashboard.preheating)
    {
        engine_control[1] |= 0x02;
    }
    if (dashboard.high_water_temp)
    {
        engine_control[1] |= 0x10;
    }
    engine_control[5] = 0;
    if (dashboard.dpf_warning)
    {
        engine_control[5] = 0x02;
    }

    int speed_prescaled =
        static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.007f);
    byte speed_low = speed_prescaled & 0xFF,
         speed_high = (speed_prescaled >> 8) & 0xFF;

    int abs_speed_prescaled =
        static_cast<unsigned short>(static_cast<float>(dashboard.speed) / 0.01f);
    byte abs_speed_low = speed_prescaled & 0xFF,
         abs_speed_high = (speed_prescaled >> 8) & 0xFF;
    byte abs_speed15_low = (speed_prescaled << 1) & 0xFF,
         abs_speed15_high = (speed_prescaled >> 7) & 0xFF;

    write_buffer[3].data[2] = dashboard.seat_belt_warning ? 0x04 : 0;

    auto &motor_speed = write_buffer[5].data;
    motor_speed[3] = abs_speed_low;
    motor_speed[4] = abs_speed_high;
    motor_speed[5] = abs_speed_low;
    motor_speed[6] = abs_speed_high;

    // RPM
    unsigned short rpm = dashboard.rpm * 4;
    write_buffer[6].data[2] = rpm & 0xFF;
    write_buffer[6].data[3] = (rpm >> 8) & 0xFF;

    // speed and drive mode
    auto &drive = write_buffer[7].data;
    drive[1] = speed_low;
    drive[2] = speed_high;
    drive[3] = 0;
    if (dashboard.abs)
    {
        drive[3] |= 0x01;
    }
    if (dashboard.offroad)
    {
        drive[3] |= 0x02;
    }
    if (dashboard.handbrake)
    {
        drive[3] |= 0x04;
    }
    if (dashboard.low_tire_pressure)
    {
        drive[3] |= 0x08;
    }

    // ABS1: has to be sent to apply the speed, though it can all be zeros?
    auto &abs1 = write_buffer[8].data;
    abs1[2] = abs_speed15_low;
    abs1[3] = abs_speed15_high;

    // ABS2: doesn't affect the dashboard?
    auto &abs2 = write_buffer[9].data;
    abs2[0] = abs_speed15_low;
    abs2[1] = abs_speed15_high;
    abs2[2] = abs_speed15_low;
    abs2[3] = abs_speed15_high;
    abs2[4] = abs_speed15_low;
    abs2[5] = abs_speed15_high;
    abs2[6] = abs_speed15_low;
    abs2[7] = abs_speed15_high;
}

void updatePins()
{
    digitalWrite(FOG_LIGHT_INDICATOR_PIN, dashboard.fog_light ? HIGH : LOW);
    digitalWrite(HIGH_BEAM_INDICATOR_PIN, dashboard.high_beam ? HIGH : LOW);
    digitalWrite(PARKING_BRAKE_CONTROL_PIN, dashboard.parking_brake ? LOW : HIGH);
}

ISR(TIMER1_COMPA_vect)
{
    if (buffer_sent)
    {
        if (write_buffer_ready)
        {
            swapBuffers();
            write_buffer_ready = false;
            buffer_sent = false;
            return;
        }
        else
        {
            send_buffer_ready = false;
            buffer_sent = false;
        }
    }
    else if (write_buffer_ready)
    {
        return;
    }

    fillPacketBuffer();

    if (send_buffer_ready)
    {
        write_buffer_ready = true;
    }
    else
    {
        swapBuffers();
        send_buffer_ready = true;
    }
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
        if (str[i] == ':')
        {
            return i;
        }
    }
    return -1;
}

void sendPackets()
{
    if (!send_buffer_ready || buffer_sent)
    {
        return;
    }
    canSend(send_buffer[0]);
    canSend(send_buffer[1]);
    canSend(send_buffer[2]);
    canSend(send_buffer[3]);
    canSend(send_buffer[4]);
    canSend(send_buffer[5]); // possibly not needed
    canSend(send_buffer[6]);
    canSend(send_buffer[7]);
    canSend(send_buffer[8]);
    canSend(send_buffer[9]); // possibly not needed
    buffer_sent = true;
    updatePins();
}

void processSerialCommand()
{
    if (Serial.available())
    {
        int len = Serial.readBytesUntil(';', serial_buffer, SERIAL_BUFFER_SIZE);
        if (len < 2)
        {
            return;
        }
        --len;
        switch (serial_buffer[0])
        {
        case 'A': // pause (bool)
            dashboard.trunk_open_warning = dashboard.door_open_warning =
                serial_buffer[1] == '1';
            break;
        case 'B': // rpm:max_rpm (int)
        {
            const int separator_index = getSeparatorIndex(serial_buffer + 1, len);
            if (separator_index == 0)
            {
                break;
            }
            const bool no_max_rpm =
                separator_index == (len - 1) || separator_index == -1;
            int rpm = str2int(serial_buffer + 1,
                              (separator_index == -1) ? len : separator_index);
#ifdef RESCALE_RPM
            if (!no_max_rpm)
            {
                const int max_rpm = str2int(serial_buffer + 2 + separator_index,
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
            dashboard.rpm = rpm;
            break;
        }
        case 'C': // speed:max_speed (int)
        {
            const int separator_index = getSeparatorIndex(serial_buffer + 1, len);
            if (separator_index == 0)
            {
                break;
            }
            const bool no_max_speed =
                separator_index == (len - 1) || separator_index == -1;
            int speed = str2int(serial_buffer + 1,
                                (separator_index == -1) ? len : separator_index);
#ifdef RESCALE_KMH
            if (!no_max_speed)
            {
                const int max_speed = str2int(serial_buffer + 2 + separator_index,
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
            dashboard.speed = speed;
            break;
        }
        case 'D': // abs (bool)
            dashboard.abs = serial_buffer[1] == '1';
            break;
        case 'E': // handbrake (bool)
            dashboard.handbrake = serial_buffer[1] == '1';
            break;
        case 'F': // parking_brake (bool)
            dashboard.parking_brake = serial_buffer[1] == '1';
            break;
        case 'G': // turn_left (bool)
        {
            bool turn_left = serial_buffer[1] == '1';
            bool turn_right = dashboard.turning_lights & 0x02;
            if (turn_left)
            {
                dashboard.turning_lights = turn_right ? 3 : 1;
            }
            else
            {
                dashboard.turning_lights = turn_right ? 2 : 0;
            }
            break;
        }
        case 'H': // turn_right (bool)
        {
            bool turn_left = dashboard.turning_lights & 0x01;
            bool turn_right = serial_buffer[1] == '1';
            if (turn_left)
            {
                dashboard.turning_lights = turn_right ? 3 : 1;
            }
            else
            {
                dashboard.turning_lights = turn_right ? 2 : 0;
            }
            break;
        }
        case 'I': // high_beam (bool)
            dashboard.high_beam = serial_buffer[1] == '1';
            break;
        case 'J': // battery_voltage (bool)
            dashboard.battery_warning = serial_buffer[1] == '1';
            break;
        case 'K': // water_temperature (bool)
            dashboard.high_water_temp =
                serial_buffer[1] == '1'; // loud AF, don't use it please
            break;
        case 'L': // backlight (bool)
            dashboard.backlight = serial_buffer[1] ==
                                  '1'; // I prefer it with backlight being always on!
            break;
        default:
            return;
        }
    }
}

void setup()
{
    pinMode(PARKING_BRAKE_CONTROL_PIN, OUTPUT);
    pinMode(FOG_LIGHT_INDICATOR_PIN, OUTPUT);
    pinMode(HIGH_BEAM_INDICATOR_PIN, OUTPUT);

    // configure 16-bit timer
    cli();                           // Disable interrupts
    TCNT1 = 0;                       // Initialize timer value with 0
    TCCR1A = 0;                      // Disable timer output pins, zero WGM11:0 for CTC mode with OCR1A
    TCCR1B = 1 << WGM12 | 1 << CS11; // Set WGM12 for CTC mode with OCR1A, set
                                     // CS12:0 to prescaler of 64
    OCR1A = static_cast<unsigned short>(
        12499U);          // So that we get frequency of 10Hz=16'000'000/(64*(24999+1)),
                          // can be changed to 12499 for 20Hz
    TIMSK1 = 1 << OCIE1A; // Enable timer interrupt A
    sei();                // Enable interrupts

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
}

void loop()
{
    sendPackets();
    processSerialCommand();
}
