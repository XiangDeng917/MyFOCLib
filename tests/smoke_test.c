#include <assert.h>
#include <stdio.h>

#include "foc.h"

static u32 fake_tick = 0;
static u16 fake_encoder = 0;

b8 motor_enable_fn(void)
{
    return 1;
}

b8 motor_disable_fn(void)
{
    return 1;
}

u32 getSysTickVal(void)
{
    fake_tick += 900;
    return fake_tick;
}

u32 getSysTickLoad(void)
{
    return 0x00FFFFFFu;
}

void setPWM(float duty_a, float duty_b, float duty_c)
{
    (void)duty_a;
    (void)duty_b;
    (void)duty_c;
}

u16 getRawCount(void)
{
    fake_encoder = (u16)((fake_encoder + 16u) % 4096u);
    return fake_encoder;
}

int main(void)
{
    BLDCMotor_s motor = {
        .pole_pairs = 7,
        .voltage_power_supply = 12.0f,
        .voltage_sensor_align = 2.0f,
        .voltage_limit = 6.0f,
    };

    PID_init();
    pid_velocity.target = 1.0f;
    (void)PID_update(&pid_velocity, 0.2f);

    setPhaseVoltage(1.0f, 0.0f, 0.0f, &motor);
    setPhaseVoltage(0.5f, 0.2f, 1.0f, &motor);

    assert(_normalizeAngle(-0.5f) >= 0.0f);
    printf("FOC smoke test passed.\n");
    return 0;
}
