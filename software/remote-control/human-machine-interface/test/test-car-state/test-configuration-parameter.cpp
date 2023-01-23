#ifdef UNIT_TEST

#include <servo-state.hpp>
#include <unity.h>

void testConfigurationParameter() {
    using earth_rover_hmi::ConfigurationParameter;

    // Create a ConfigurationParameter<int>, using int's default constructor.
    // Value: 0, EEPROM value available: false, current value stored: true (default value).
    ConfigurationParameter<int> configuration_parameter_1;
    TEST_ASSERT_EQUAL(0, configuration_parameter_1.getCurrentConfiguration());
    TEST_ASSERT_FALSE(configuration_parameter_1.isConfigurationAvailable());
    TEST_ASSERT_TRUE(configuration_parameter_1.isCurrentConfigurationStored());
    // Restore value 0 from EEPROM.
    // Value: 0, EEPROM value available: true, current value stored: true.
    configuration_parameter_1.setStoredConfiguration(0);
    TEST_ASSERT_EQUAL(0, configuration_parameter_1.getCurrentConfiguration());
    TEST_ASSERT_TRUE(configuration_parameter_1.isConfigurationAvailable());
    TEST_ASSERT_TRUE(configuration_parameter_1.isCurrentConfigurationStored());
    // Set current value to 1.
    // Value: 1, EEPROM value available: true, current value stored: false.
    configuration_parameter_1.setCurrentConfiguration(1);
    TEST_ASSERT_EQUAL(1, configuration_parameter_1.getCurrentConfiguration());
    TEST_ASSERT_TRUE(configuration_parameter_1.isConfigurationAvailable());
    TEST_ASSERT_FALSE(configuration_parameter_1.isCurrentConfigurationStored());
    // Store current value.
    // Value: 1, EEPROM value available: true, current value stored: true.
    configuration_parameter_1.setCurrentConfigurationStored();
    TEST_ASSERT_EQUAL(1, configuration_parameter_1.getCurrentConfiguration());
    TEST_ASSERT_TRUE(configuration_parameter_1.isConfigurationAvailable());
    TEST_ASSERT_TRUE(configuration_parameter_1.isCurrentConfigurationStored());
    // Create a ConfigurationParameter<int>, using an explicit default value.
    // Value: 2, EEPROM value available: false, current value stored: true (default value).
    ConfigurationParameter<int> configuration_parameter_2{2};
    TEST_ASSERT_EQUAL(2, configuration_parameter_2.getCurrentConfiguration());
    TEST_ASSERT_FALSE(configuration_parameter_2.isConfigurationAvailable());
    TEST_ASSERT_TRUE(configuration_parameter_2.isCurrentConfigurationStored());
}

#endif