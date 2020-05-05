#ifdef UNIT_TEST

#include <servo-state.hpp>
#include <unity.h>

void testServoConfigParams() {
    using earth_rover_hmi::ServoConfigParams;

    uint8_t input_channel = 2u;
    uint16_t pulse_width_minimum = 1000u;
    uint16_t pulse_width_center = 1500u;
    uint16_t pulse_width_maximum = 2000u;
    bool enforce_pulse_width_limits = true;

    ServoConfigParams servo_config_1{input_channel, pulse_width_minimum, pulse_width_center,
                                     pulse_width_maximum, enforce_pulse_width_limits};

    TEST_ASSERT_EQUAL_UINT8(input_channel, servo_config_1.input_channel);
    TEST_ASSERT_EQUAL_UINT16(pulse_width_minimum, servo_config_1.pulse_width_minimum);
    TEST_ASSERT_EQUAL_UINT16(pulse_width_center, servo_config_1.pulse_width_center);
    TEST_ASSERT_EQUAL_UINT16(pulse_width_maximum, servo_config_1.pulse_width_maximum);
    TEST_ASSERT_EQUAL(enforce_pulse_width_limits, servo_config_1.enforce_pulse_width_limits);

    TEST_ASSERT_FALSE(servo_config_1 != servo_config_1);

    uint8_t input_channel_ne = input_channel + 1u;
    uint16_t pulse_width_minimum_ne = pulse_width_minimum + 100u;
    uint16_t pulse_width_center_ne = pulse_width_center + 100u;
    uint16_t pulse_width_maximum_ne = pulse_width_maximum + 100u;
    bool enforce_pulse_width_limits_ne = !enforce_pulse_width_limits;

    ServoConfigParams servo_config_2{input_channel_ne, pulse_width_minimum, pulse_width_center,
                                     pulse_width_maximum, enforce_pulse_width_limits};
    TEST_ASSERT_TRUE(servo_config_1 != servo_config_2);

    ServoConfigParams servo_config_3{input_channel, pulse_width_minimum_ne, pulse_width_center,
                                     pulse_width_maximum, enforce_pulse_width_limits};
    TEST_ASSERT_TRUE(servo_config_1 != servo_config_3);

    ServoConfigParams servo_config_4{input_channel, pulse_width_minimum, pulse_width_center_ne,
                                     pulse_width_maximum, enforce_pulse_width_limits};
    TEST_ASSERT_TRUE(servo_config_1 != servo_config_4);

    ServoConfigParams servo_config_5{input_channel, pulse_width_minimum, pulse_width_center,
                                     pulse_width_maximum_ne, enforce_pulse_width_limits};
    TEST_ASSERT_TRUE(servo_config_1 != servo_config_5);

    ServoConfigParams servo_config_6{input_channel, pulse_width_minimum, pulse_width_center,
                                     pulse_width_maximum, enforce_pulse_width_limits_ne};
    TEST_ASSERT_TRUE(servo_config_1 != servo_config_6);
}

void testServoState() {
    using earth_rover_hmi::ServoConfigParams;
    using earth_rover_hmi::ServoState;

    uint8_t input_channel = 1u;
    uint16_t pulse_width_minimum = 1000u;
    uint16_t pulse_width_center = 1500u;
    uint16_t pulse_width_maximum = 2000u;
    bool enforce_pulse_width_limits = true;

    ServoState servo_state_1{ServoConfigParams{input_channel, pulse_width_minimum,
                                               pulse_width_center, pulse_width_maximum,
                                               enforce_pulse_width_limits}};
    TEST_ASSERT_EQUAL_INT16(0, servo_state_1.getCurrentPosition());
    servo_state_1.setCurrentPosition(-1001);
    TEST_ASSERT_EQUAL_INT16(-1000, servo_state_1.getCurrentPosition());
    servo_state_1.setCurrentPosition(-1000);
    TEST_ASSERT_EQUAL_INT16(-1000, servo_state_1.getCurrentPosition());
    servo_state_1.setCurrentPosition(-999);
    TEST_ASSERT_EQUAL_INT16(-999, servo_state_1.getCurrentPosition());
    servo_state_1.setCurrentPosition(0);
    TEST_ASSERT_EQUAL_INT16(0, servo_state_1.getCurrentPosition());
    servo_state_1.setCurrentPosition(999);
    TEST_ASSERT_EQUAL_INT16(999, servo_state_1.getCurrentPosition());
    servo_state_1.setCurrentPosition(1000);
    TEST_ASSERT_EQUAL_INT16(1000, servo_state_1.getCurrentPosition());
    servo_state_1.setCurrentPosition(1001);
    TEST_ASSERT_EQUAL_INT16(1000, servo_state_1.getCurrentPosition());
}

#endif