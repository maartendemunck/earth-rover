#ifdef UNIT_TEST

#include <gearbox-servo-state.hpp>
#include <unity.h>

void testGearboxServoConfigParams() {
    using earth_rover_hmi::GearboxServoConfigParams;

    // Define pulse widths for different gears.
    uint16_t pulse_width_r1{1000};
    uint16_t pulse_width_n{1500};
    uint16_t pulse_width_f1{1750};
    uint16_t pulse_width_f2{2000};

    // Test a GearboxServoConfigParams with 2 forward gears.
    {
        uint16_t gearbox_pulse_widths[2]{pulse_width_f1, pulse_width_f2};
        GearboxServoConfigParams<0, false, 2> gearbox_config(1u, gearbox_pulse_widths);
        TEST_ASSERT_EQUAL_UINT16(pulse_width_f1, gearbox_config.getPulseWidth(1));
        TEST_ASSERT_EQUAL_UINT16(pulse_width_f2, gearbox_config.getPulseWidth(2));
        for(int gear = 1; gear <= 2; ++gear) {
            uint16_t new_pulse_width = gearbox_config.getPulseWidth(gear) + 100;
            gearbox_config.setPulseWidth(gear, new_pulse_width);
            TEST_ASSERT_EQUAL_UINT16(new_pulse_width, gearbox_config.getPulseWidth(gear));
        }
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(-1));
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(0));
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(3));
    }

    // Test a GearboxServoConfigParams with a neutral and 2 forward gears.
    {
        uint16_t gearbox_pulse_widths[3]{pulse_width_n, pulse_width_f1, pulse_width_f2};
        GearboxServoConfigParams<0, true, 2> gearbox_config(1u, gearbox_pulse_widths);
        TEST_ASSERT_EQUAL_UINT16(pulse_width_n, gearbox_config.getPulseWidth(0));
        TEST_ASSERT_EQUAL_UINT16(pulse_width_f1, gearbox_config.getPulseWidth(1));
        TEST_ASSERT_EQUAL_UINT16(pulse_width_f2, gearbox_config.getPulseWidth(2));
        for(int gear = 0; gear <= 2; ++gear) {
            uint16_t new_pulse_width = gearbox_config.getPulseWidth(gear) + 100;
            gearbox_config.setPulseWidth(gear, new_pulse_width);
            TEST_ASSERT_EQUAL_UINT16(new_pulse_width, gearbox_config.getPulseWidth(gear));
        }
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(-1));
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(3));
    }

    // Test a GearboxServoConfigParams with a reverse and a forward gear.
    {
        uint16_t gearbox_pulse_widths[2]{pulse_width_r1, pulse_width_f1};
        GearboxServoConfigParams<1, false, 1> gearbox_config(1u, gearbox_pulse_widths);
        TEST_ASSERT_EQUAL_UINT16(pulse_width_r1, gearbox_config.getPulseWidth(-1));
        TEST_ASSERT_EQUAL_UINT16(pulse_width_f1, gearbox_config.getPulseWidth(1));
        for(int gear = -1; gear <= 1; ++gear) {
            if(gear == 0) {
                continue;
            }
            uint16_t new_pulse_width = gearbox_config.getPulseWidth(gear) + 100;
            gearbox_config.setPulseWidth(gear, new_pulse_width);
            TEST_ASSERT_EQUAL_UINT16(new_pulse_width, gearbox_config.getPulseWidth(gear));
        }
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(-2));
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(0));
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(2));
    }

    // Test a GearboxServoConfigParams with a reverse, a neutral and a forward gear.
    {
        uint16_t gearbox_pulse_widths[3]{pulse_width_r1, pulse_width_n, pulse_width_f1};
        GearboxServoConfigParams<1, true, 1> gearbox_config(1u, gearbox_pulse_widths);
        TEST_ASSERT_EQUAL_UINT16(pulse_width_r1, gearbox_config.getPulseWidth(-1));
        TEST_ASSERT_EQUAL_UINT16(pulse_width_n, gearbox_config.getPulseWidth(0));
        TEST_ASSERT_EQUAL_UINT16(pulse_width_f1, gearbox_config.getPulseWidth(1));
        for(int gear = -1; gear <= 1; ++gear) {
            uint16_t new_pulse_width = gearbox_config.getPulseWidth(gear) + 100;
            gearbox_config.setPulseWidth(gear, new_pulse_width);
            TEST_ASSERT_EQUAL_UINT16(new_pulse_width, gearbox_config.getPulseWidth(gear));
        }
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(-2));
        TEST_ASSERT_EQUAL_UINT16(0, gearbox_config.getPulseWidth(2));
    }
}

void testGearboxServoState() {
    using earth_rover_hmi::GearboxServoConfigParams;
    using earth_rover_hmi::GearboxServoState;

    // Define pulse widths for different gears.
    uint16_t pulse_width_r1{1000};
    uint16_t pulse_width_n{1500};
    uint16_t pulse_width_f1{1750};
    uint16_t pulse_width_f2{2000};

    // Test a GearboxServoState with 2 forward gears.
    {
        // Create gearbox state.
        uint16_t gearbox_pulse_widths[2]{pulse_width_f1, pulse_width_f2};
        GearboxServoConfigParams<0, false, 2> gearbox_config(1u, gearbox_pulse_widths);
        GearboxServoState<0, false, 2> gearbox_state(gearbox_config);
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());  // Start in first gear.
        // Test setting gears directly.
        for(int gear = 1; gear <= 2; ++gear) {
            gearbox_state.setCurrentGear(gear);
            TEST_ASSERT_EQUAL_INT8(gear, gearbox_state.getCurrentGear());
        }
        gearbox_state.setCurrentGear(1);
        gearbox_state.setCurrentGear(0);  // Gearbox doesn't have a neutral gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.setCurrentGear(3);  // Gearbox doesn't have a third gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        // Test shifting through gears up and down.
        gearbox_state.setCurrentGear(1);  // Start in first gear.
        gearbox_state.shiftUp();  // Shift to second gear.
        TEST_ASSERT_EQUAL_INT8(2, gearbox_state.getCurrentGear());
        gearbox_state.shiftUp();  // No third gear.
        TEST_ASSERT_EQUAL_INT8(2, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // Shift to first gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // No neutral gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
    }

    // Test a GearboxServoState with a neutral and 2 forward gears.
    {
        // Create gearbox state.
        uint16_t gearbox_pulse_widths[3]{pulse_width_n, pulse_width_f1, pulse_width_f2};
        GearboxServoConfigParams<0, true, 2> gearbox_config(1u, gearbox_pulse_widths);
        GearboxServoState<0, true, 2> gearbox_state(gearbox_config);
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());  // Start in neutral gear.
        // Test setting gears directly.
        for(int gear = 0; gear <= 2; ++gear) {
            gearbox_state.setCurrentGear(gear);
            TEST_ASSERT_EQUAL_INT8(gear, gearbox_state.getCurrentGear());
        }
        gearbox_state.setCurrentGear(0);
        gearbox_state.setCurrentGear(-1);  // Gearbox doesn't have a reverse gear.
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());
        gearbox_state.setCurrentGear(3);  // Gearbox doesn't have a third gear.
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());
        // Test shifting through gears up and down.
        gearbox_state.setCurrentGear(0);  // Start in neutral gear.
        gearbox_state.shiftUp();  // Shift to first gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.shiftUp();  // Shift to second gear.
        TEST_ASSERT_EQUAL_INT8(2, gearbox_state.getCurrentGear());
        gearbox_state.shiftUp();  // No third gear.
        TEST_ASSERT_EQUAL_INT8(2, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // Shift to first gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // Shift to neutral gear.
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // No reverse gear.
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());
    }

    // Test a GearboxServoState with a reverse and a forward gear.
    {
        // Create gearbox state.
        uint16_t gearbox_pulse_widths[2]{pulse_width_r1, pulse_width_f1};
        GearboxServoConfigParams<1, false, 1> gearbox_config(1u, gearbox_pulse_widths);
        GearboxServoState<1, false, 1> gearbox_state(gearbox_config);
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());  // Start in forward gear.
        // Test setting gears directly.
        for(int gear = -1; gear <= 1; ++gear) {
            if(gear == 0) {  // Gearbox doesn't have a neutral gear.
                continue;
            }
            gearbox_state.setCurrentGear(gear);
            TEST_ASSERT_EQUAL_INT8(gear, gearbox_state.getCurrentGear());
        }
        gearbox_state.setCurrentGear(1);
        gearbox_state.setCurrentGear(0);  // Gearbox doesn't have a neutral gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.setCurrentGear(-2);  // Gearbox doesn't have a second reverse gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.setCurrentGear(2);  // Gearbox doesn't have a second gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        // Test shifting through gears up and down.
        gearbox_state.setCurrentGear(-1);  // Start in reverse gear.
        gearbox_state.shiftUp();  // Shift to first gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.shiftUp();  // No second gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // Shift to reverse gear.
        TEST_ASSERT_EQUAL_INT8(-1, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // No second reverse gear.
        TEST_ASSERT_EQUAL_INT8(-1, gearbox_state.getCurrentGear());
    }

    // Test a GearboxServoState with a reverse, a neutral and a forward gear.
    {
        // Create gearbox state.
        uint16_t gearbox_pulse_widths[3]{pulse_width_r1, pulse_width_n, pulse_width_f1};
        GearboxServoConfigParams<1, true, 1> gearbox_config(1u, gearbox_pulse_widths);
        GearboxServoState<1, true, 1> gearbox_state(gearbox_config);
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());  // Start in neutral gear.
        // Test setting gears directly.
        for(int gear = -1; gear <= 1; ++gear) {
            gearbox_state.setCurrentGear(gear);
            TEST_ASSERT_EQUAL_INT8(gear, gearbox_state.getCurrentGear());
        }
        gearbox_state.setCurrentGear(0);
        gearbox_state.setCurrentGear(-2);  // Gearbox doesn't have a second reverse gear.
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());
        gearbox_state.setCurrentGear(2);  // Gearbox doesn't have a second gear.
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());
        // Test shifting through gears up and down.
        gearbox_state.setCurrentGear(-1);  // Start in reverse gear.
        gearbox_state.shiftUp();  // Shift to neutral gear.
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());
        gearbox_state.shiftUp();  // Shift to first gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.shiftUp();  // No second gear.
        TEST_ASSERT_EQUAL_INT8(1, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // Shift to neutral gear.
        TEST_ASSERT_EQUAL_INT8(0, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // Shift to reverse gear.
        TEST_ASSERT_EQUAL_INT8(-1, gearbox_state.getCurrentGear());
        gearbox_state.shiftDown();  // No second reverse gear.
        TEST_ASSERT_EQUAL_INT8(-1, gearbox_state.getCurrentGear());
    }
}

#endif