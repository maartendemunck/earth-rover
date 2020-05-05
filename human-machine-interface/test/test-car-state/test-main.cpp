#ifdef UNIT_TEST

#include "test-configuration-parameter.hpp"
#include "test-servo-state.hpp"
#include <unity.h>

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(testConfigurationParameter);
    RUN_TEST(testServoConfigParams);
    RUN_TEST(testServoState);
    UNITY_END();
}

#endif