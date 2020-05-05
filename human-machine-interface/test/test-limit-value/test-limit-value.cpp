#ifdef UNIT_TEST

#include <limit-value.hpp>
#include <unity.h>

void testLimitValue() {
    using earth_rover_hmi::limit_value;

    TEST_ASSERT_EQUAL(-1000, limit_value<int>(-1001, -1000, 1000));
    TEST_ASSERT_EQUAL(-1000, limit_value<int>(-1000, -1000, 1000));
    TEST_ASSERT_EQUAL(-999, limit_value<int>(-999, -1000, 1000));
    TEST_ASSERT_EQUAL(0, limit_value<int>(0, -1000, 1000));
    TEST_ASSERT_EQUAL(999, limit_value<int>(999, -1000, 1000));
    TEST_ASSERT_EQUAL(1000, limit_value<int>(1000, -1000, 1000));
    TEST_ASSERT_EQUAL(1000, limit_value<int>(1001, -1000, 1000));

    TEST_ASSERT_EQUAL(100, limit_value<int>(99, 100, 1000));
    TEST_ASSERT_EQUAL(100, limit_value<int>(100, 100, 1000));
    TEST_ASSERT_EQUAL(101, limit_value<int>(101, 100, 1000));
    TEST_ASSERT_EQUAL(999, limit_value<int>(999, 100, 1000));
    TEST_ASSERT_EQUAL(1000, limit_value<int>(1000, 100, 1000));
    TEST_ASSERT_EQUAL(1000, limit_value<int>(1001, 100, 1000));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(testLimitValue);
    UNITY_END();
}
#endif