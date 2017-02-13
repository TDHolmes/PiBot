#include "unity.h"
#include "../MotorController/motor_calc.h"
#include "../MotorController/constants.h"


void test_MinMacro(void)
{
    TEST_ASSERT_EQUAL_HEX8(4, MIN(4, 5));
    TEST_ASSERT_EQUAL_HEX8(4, MIN(4, 4));
    TEST_ASSERT( 4.0 == MIN(4.0, 4.01));
}


void test_MaxMacro(void)
{
    TEST_ASSERT_EQUAL_HEX8(5, MAX(4, 5));
    TEST_ASSERT_EQUAL_HEX8(4, MAX(4, 4));
    TEST_ASSERT( 4.01 == MAX(4.0, 4.01));
}


void test_VelocityCalculation(void)
{
    // Initialize the variables starting at t=0
    motor_calc_init(0);
    // the left motor went 10 ticks, the right 20, in 100 ms
    motor_calc_parameters(10, 20, 100);

    TEST_ASSERT_EQUAL_FLOAT( (10.0f / 100.0f), motor_calc_velocity_get(kMotor_Left) );
    TEST_ASSERT_EQUAL_FLOAT( (20.0f / 100.0f), motor_calc_velocity_get(kMotor_Right) );

    // the left motor went 30 ticks, the right 15, in 50 ms
    motor_calc_parameters(30, 15, 150);

    TEST_ASSERT_EQUAL_FLOAT( ((30.0f - 10.0f) / 50.0f), motor_calc_velocity_get(kMotor_Left) );
    TEST_ASSERT_EQUAL_FLOAT( ((15.0f - 20.0f) / 50.0f), motor_calc_velocity_get(kMotor_Right) );
}



void test_DistanceCalculation(void)
{
    // Initialize the variables starting at t=0
    motor_calc_init(0);
    // the left motor went 10 ticks, the right 20, in 100 ms
    motor_calc_parameters(10, 20, 100);

    TEST_ASSERT_EQUAL_UINT32( 10, motor_calc_distance_get(kMotor_Left) );
    TEST_ASSERT_EQUAL_UINT32( 20, motor_calc_distance_get(kMotor_Right) );

    // the left motor went 30 ticks, the right 15, in 50 ms
    motor_calc_parameters(30, 15, 150);

    TEST_ASSERT_EQUAL_UINT32( 30, motor_calc_distance_get(kMotor_Left) );
    TEST_ASSERT_EQUAL_UINT32( 15, motor_calc_distance_get(kMotor_Right) );
}



int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_MinMacro);
    RUN_TEST(test_MaxMacro);
    RUN_TEST(test_VelocityCalculation);
    RUN_TEST(test_DistanceCalculation);

    return UNITY_END();
}
