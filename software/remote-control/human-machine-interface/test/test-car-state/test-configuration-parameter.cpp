#ifdef UNIT_TEST

#include <servo-state.hpp>
#include <unity.h>

void testConfigParameter() {
    using earth_rover_hmi::ConfigParameter;

    // Create a ConfigParameter<int>, using int's default constructor.
    // Value: 0, EEPROM value available: false, current value saved: true (default value).
    ConfigParameter<int> config_parameter_1;
    TEST_ASSERT_EQUAL(0, config_parameter_1.getCurrentConfig());
    TEST_ASSERT_FALSE(config_parameter_1.isConfigAvailable());
    TEST_ASSERT_TRUE(config_parameter_1.isCurrentConfigSaved());
    // Restore value 0 from EEPROM.
    // Value: 0, EEPROM value available: true, current value saved: true.
    config_parameter_1.setSavedConfig(0);
    TEST_ASSERT_EQUAL(0, config_parameter_1.getCurrentConfig());
    TEST_ASSERT_TRUE(config_parameter_1.isConfigAvailable());
    TEST_ASSERT_TRUE(config_parameter_1.isCurrentConfigSaved());
    // Set current value to 1.
    // Value: 1, EEPROM value available: true, current value saved: false.
    config_parameter_1.setCurrentConfig(1);
    TEST_ASSERT_EQUAL(1, config_parameter_1.getCurrentConfig());
    TEST_ASSERT_TRUE(config_parameter_1.isConfigAvailable());
    TEST_ASSERT_FALSE(config_parameter_1.isCurrentConfigSaved());
    // Save current value.
    // Value: 1, EEPROM value available: true, current value saved: true.
    config_parameter_1.setCurrentConfigSaved();
    TEST_ASSERT_EQUAL(1, config_parameter_1.getCurrentConfig());
    TEST_ASSERT_TRUE(config_parameter_1.isConfigAvailable());
    TEST_ASSERT_TRUE(config_parameter_1.isCurrentConfigSaved());
    // Create a ConfigParameter<int>, using an explicit default value.
    // Value: 2, EEPROM value available: false, current value saved: true (default value).
    ConfigParameter<int> config_parameter_2{2};
    TEST_ASSERT_EQUAL(2, config_parameter_2.getCurrentConfig());
    TEST_ASSERT_FALSE(config_parameter_2.isConfigAvailable());
    TEST_ASSERT_TRUE(config_parameter_2.isCurrentConfigSaved());
}

#endif