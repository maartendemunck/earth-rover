#ifdef UNIT_TEST

#include <radio-state.hpp>
#include <unity.h>

void testRadioConfigParams() {
    using earth_rover_hmi::RadioConfigParams;

    RadioConfigParams radio_config_1{2u, 3u};
    TEST_ASSERT_EQUAL_UINT8(2u, radio_config_1.tx_power);
    TEST_ASSERT_EQUAL_UINT8(3u, radio_config_1.rx_power);
    TEST_ASSERT_FALSE(radio_config_1 != radio_config_1);
    RadioConfigParams radio_config_2{1u, 3u};
    TEST_ASSERT_TRUE(radio_config_1 != radio_config_2);
    RadioConfigParams radio_config_3{2u, 4u};
    TEST_ASSERT_TRUE(radio_config_1 != radio_config_3);
}

#endif