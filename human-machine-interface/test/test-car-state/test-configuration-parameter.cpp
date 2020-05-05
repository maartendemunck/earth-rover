#ifdef UNIT_TEST

#include <servo-state.hpp>
#include <unity.h>

void testConfigurationParameter() {
    using earth_rover_hmi::ConfigurationParameter;

    ConfigurationParameter<int> configuration_parameter_1;
    TEST_ASSERT_EQUAL(0, configuration_parameter_1.getCurrentConfiguration());
    TEST_ASSERT_FALSE(configuration_parameter_1.isConfigurationAvailable());
    TEST_ASSERT_TRUE(configuration_parameter_1.isCurrentConfigurationStored());
    configuration_parameter_1.setStoredConfiguration(0);
    TEST_ASSERT_EQUAL(0, configuration_parameter_1.getCurrentConfiguration());
    TEST_ASSERT_TRUE(configuration_parameter_1.isConfigurationAvailable());
    TEST_ASSERT_TRUE(configuration_parameter_1.isCurrentConfigurationStored());
    configuration_parameter_1.setCurrentConfiguration(1);
    TEST_ASSERT_EQUAL(1, configuration_parameter_1.getCurrentConfiguration());
    TEST_ASSERT_TRUE(configuration_parameter_1.isConfigurationAvailable());
    TEST_ASSERT_FALSE(configuration_parameter_1.isCurrentConfigurationStored());
    configuration_parameter_1.setCurrentConfigurationStored();
    TEST_ASSERT_EQUAL(1, configuration_parameter_1.getCurrentConfiguration());
    TEST_ASSERT_TRUE(configuration_parameter_1.isConfigurationAvailable());
    TEST_ASSERT_TRUE(configuration_parameter_1.isCurrentConfigurationStored());
}

#endif