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

    ServoConfigParams ServoConfig1{input_channel, pulse_width_minimum, pulse_width_center,
                                   pulse_width_maximum, enforce_pulse_width_limits};

    TEST_ASSERT_EQUAL_UINT8(input_channel, ServoConfig1.input_channel);
    TEST_ASSERT_EQUAL_UINT16(pulse_width_minimum, ServoConfig1.pulse_width_minimum);
    TEST_ASSERT_EQUAL_UINT16(pulse_width_center, ServoConfig1.pulse_width_center);
    TEST_ASSERT_EQUAL_UINT16(pulse_width_maximum, ServoConfig1.pulse_width_maximum);
    TEST_ASSERT_EQUAL(enforce_pulse_width_limits, ServoConfig1.enforce_pulse_width_limits);

    TEST_ASSERT_FALSE(ServoConfig1 != ServoConfig1);

    uint8_t input_channel_ne = input_channel + 1u;
    uint16_t pulse_width_minimum_ne = pulse_width_minimum + 100u;
    uint16_t pulse_width_center_ne = pulse_width_center + 100u;
    uint16_t pulse_width_maximum_ne = pulse_width_maximum + 100u;
    bool enforce_pulse_width_limits_ne = !enforce_pulse_width_limits;

    ServoConfigParams ServoConfig2{input_channel_ne, pulse_width_minimum, pulse_width_center,
                                   pulse_width_maximum, enforce_pulse_width_limits};
    TEST_ASSERT_TRUE(ServoConfig1 != ServoConfig2);

    ServoConfigParams ServoConfig3{input_channel, pulse_width_minimum_ne, pulse_width_center,
                                   pulse_width_maximum, enforce_pulse_width_limits};
    TEST_ASSERT_TRUE(ServoConfig1 != ServoConfig3);

    ServoConfigParams ServoConfig4{input_channel, pulse_width_minimum, pulse_width_center_ne,
                                   pulse_width_maximum, enforce_pulse_width_limits};
    TEST_ASSERT_TRUE(ServoConfig1 != ServoConfig4);

    ServoConfigParams ServoConfig5{input_channel, pulse_width_minimum, pulse_width_center,
                                   pulse_width_maximum_ne, enforce_pulse_width_limits};
    TEST_ASSERT_TRUE(ServoConfig1 != ServoConfig5);

    ServoConfigParams ServoConfig6{input_channel, pulse_width_minimum, pulse_width_center,
                                   pulse_width_maximum, enforce_pulse_width_limits_ne};
    TEST_ASSERT_TRUE(ServoConfig1 != ServoConfig6);
}

void testServoState() {
    using earth_rover_hmi::ServoConfigParams;
    using earth_rover_hmi::ServoState;

    uint8_t input_channel = 1u;
    uint16_t pulse_width_minimum = 1000u;
    uint16_t pulse_width_center = 1500u;
    uint16_t pulse_width_maximum = 2000u;
    bool enforce_pulse_width_limits = true;

    ServoState ServoState1{ServoConfigParams{input_channel, pulse_width_minimum, pulse_width_center,
                                             pulse_width_maximum, enforce_pulse_width_limits}};
    TEST_ASSERT_EQUAL_INT16(0, ServoState1.getCurrentPosition());
    ServoState1.setCurrentPosition(-1001);
    TEST_ASSERT_EQUAL_INT16(-1000, ServoState1.getCurrentPosition());
    ServoState1.setCurrentPosition(-1000);
    TEST_ASSERT_EQUAL_INT16(-1000, ServoState1.getCurrentPosition());
    ServoState1.setCurrentPosition(-999);
    TEST_ASSERT_EQUAL_INT16(-999, ServoState1.getCurrentPosition());
    ServoState1.setCurrentPosition(0);
    TEST_ASSERT_EQUAL_INT16(0, ServoState1.getCurrentPosition());
    ServoState1.setCurrentPosition(999);
    TEST_ASSERT_EQUAL_INT16(999, ServoState1.getCurrentPosition());
    ServoState1.setCurrentPosition(1000);
    TEST_ASSERT_EQUAL_INT16(1000, ServoState1.getCurrentPosition());
    ServoState1.setCurrentPosition(1001);
    TEST_ASSERT_EQUAL_INT16(1000, ServoState1.getCurrentPosition());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    testServoConfigParams();
    testServoState();
    UNITY_END();
}

#endif