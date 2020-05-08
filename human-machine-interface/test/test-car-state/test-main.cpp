#ifdef UNIT_TEST

#include "test-configuration-parameter.hpp"
#include "test-gearbox-servo-state.hpp"
#include "test-radio-state.hpp"
#include "test-servo-state.hpp"
#include <unity.h>

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(testConfigurationParameter);
    RUN_TEST(testServoConfigParams);
    RUN_TEST(testServoState);
    RUN_TEST(testGearboxServoConfigParams);
    RUN_TEST(testGearboxServoState);
    RUN_TEST(testRadioConfigParams);
    UNITY_END();
}

#endif