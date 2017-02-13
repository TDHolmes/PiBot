#include "unity.h"
#include <stdio.h>
#include "../MotorController/motor_calc.h"
#include "../MotorController/constants.h"


/*****************************************************************************/
/* Min Max Macro tests                                                       */
/*****************************************************************************/


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


/*****************************************************************************/
/* Velocity and Distance Calculation Tests                                   */
/*****************************************************************************/


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


void test_PIDmodeSet(void)
{
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_velocity);
    TEST_ASSERT( kPID_velocity == motor_calc_PID_getmode() );
    motor_calc_PID_setmode(kPID_distance);
    TEST_ASSERT( kPID_distance == motor_calc_PID_getmode() );
    motor_calc_PID_setmode(kPID_stop);
    TEST_ASSERT( kPID_stop == motor_calc_PID_getmode() );
}


void test_MotorCalc_NoINF(void)
{
    motor_calc_init(0);
    // Call it with 30 ticks moved in no time
    motor_calc_parameters(30, 30, 0);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("velocity Left: %f\n", motor_calc_velocity_get(kMotor_Left));
    printf("velocity Right: %f\n", motor_calc_velocity_get(kMotor_Right));
    printf("distance Left: %d\n", motor_calc_distance_get(kMotor_Left));
    printf("distance Right: %d\n", motor_calc_distance_get(kMotor_Right));
    printf("\n");
    #endif

    // Make sure velocity and distance is still 0
    TEST_ASSERT_EQUAL_FLOAT(0, motor_calc_velocity_get(kMotor_Left));
    TEST_ASSERT_EQUAL_FLOAT(0, motor_calc_velocity_get(kMotor_Right));
    TEST_ASSERT_EQUAL_INT32(0, motor_calc_distance_get(kMotor_Left));
    TEST_ASSERT_EQUAL_INT32(0, motor_calc_distance_get(kMotor_Right));
}


/*****************************************************************************/
/* PID Velocity tests                                                        */
/*****************************************************************************/


void test_PIDrunVelocity_ZeroError_WhenTargetMet(void)
{
    float vel_target = 0.2f;
    float err_output_l = 0;
    float err_output_r = 0;
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_velocity);
    motor_calc_PID_set_target(kMotor_Left, &vel_target);
    motor_calc_PID_set_target(kMotor_Right, &vel_target);
    motor_calc_parameters(20, 20, 100);
    motor_calc_PID_run(&err_output_l, &err_output_r);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("\tError Output Left:  %f\n", err_output_l);
    printf("\tError Output Right: %f\n", err_output_r);
    printf("\n");
    #endif

    TEST_ASSERT_EQUAL_FLOAT(0.0, err_output_r);
    TEST_ASSERT_EQUAL_FLOAT(0.0, err_output_l);
}


void test_PIDrunVelocity_NegativeError_WhenOverTarget(void)
{
    float vel_target = 0.2f;
    float err_output_l = 0;
    float err_output_r = 0;
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_velocity);
    motor_calc_PID_set_target(kMotor_Left, &vel_target);
    motor_calc_PID_set_target(kMotor_Right, &vel_target);
    motor_calc_parameters(30, 30, 100);
    motor_calc_PID_run(&err_output_l, &err_output_r);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("\tError Output Left:  %f\n", err_output_l);
    printf("\tError Output Right: %f\n", err_output_r);
    printf("\n");
    #endif

    TEST_ASSERT(0.0 > err_output_r);
    TEST_ASSERT(0.0 > err_output_l);
}


void test_PIDrunVelocity_PositiveError_WhenUnderTarget(void)
{
    float vel_target = 0.2f;
    float err_output_l = 0;
    float err_output_r = 0;
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_velocity);
    motor_calc_PID_set_target(kMotor_Left, &vel_target);
    motor_calc_PID_set_target(kMotor_Right, &vel_target);
    motor_calc_parameters(10, 10, 100);
    motor_calc_PID_run(&err_output_l, &err_output_r);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("\tError Output Left:  %f\n", err_output_l);
    printf("\tError Output Right: %f\n", err_output_r);
    printf("\n");
    #endif

    TEST_ASSERT(0.0 < err_output_r);
    TEST_ASSERT(0.0 < err_output_l);
}


void test_PIDrunVelocity_ErrorGrows_IfNoProgress(void)
{
    float vel_target = 2.0f;
    float first_err_output_l = 0;
    float first_err_output_r = 0;
    float second_err_output_l = 0;
    float second_err_output_r = 0;

    // setup initial conditions...
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_velocity);
    motor_calc_PID_set_target(kMotor_Left, &vel_target);
    motor_calc_PID_set_target(kMotor_Right, &vel_target);

    // imitate the first cycle (no movement over 100 ticks)
    motor_calc_parameters(0, 0, 100);
    motor_calc_PID_run(&first_err_output_l, &first_err_output_r);

    // imitate some small movement after 100 more ticks
    motor_calc_parameters(0, 0, 200);
    motor_calc_PID_run(&second_err_output_l, &second_err_output_r);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("\tVelocity Target: %f\tVelocity: %f\n", vel_target, motor_calc_velocity_get(kMotor_Right));
    printf("\tError Output: %f\n", first_err_output_l);
    printf("\tVelocity Target: %f\tVelocity: %f\n", vel_target, motor_calc_velocity_get(kMotor_Right));
    printf("\tError Output: %f\n", second_err_output_l);
    printf("\n");
    #endif


    // the error output should be slightly more the second time (integral error)
    TEST_ASSERT(first_err_output_l < second_err_output_l);
    TEST_ASSERT(first_err_output_r < second_err_output_r);
}


/*****************************************************************************/
/* PID Distance tests                                                        */
/*****************************************************************************/


void test_PIDrunDistance_ZeroError_WhenTargetMet(void)
{
    int32_t distance_target = 30;
    float err_output_l = 0;
    float err_output_r = 0;
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_distance);
    motor_calc_PID_set_target(kMotor_Left, &distance_target);
    motor_calc_PID_set_target(kMotor_Right, &distance_target);
    motor_calc_parameters(30, 30, 100);
    motor_calc_PID_run(&err_output_l, &err_output_r);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("\tError Output Left:  %f\n", err_output_l);
    printf("\tError Output Right: %f\n", err_output_r);
    printf("\n");
    #endif

    TEST_ASSERT_EQUAL_FLOAT(0.0, err_output_r);
    TEST_ASSERT_EQUAL_FLOAT(0.0, err_output_l);
}


void test_PIDrunDistance_NegativeError_WhenOverTarget(void)
{
    int32_t distance_target = 30;
    float err_output_l = 0;
    float err_output_r = 0;
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_distance);
    motor_calc_PID_set_target(kMotor_Left, &distance_target);
    motor_calc_PID_set_target(kMotor_Right, &distance_target);
    motor_calc_parameters(40, 40, 100);
    motor_calc_PID_run(&err_output_l, &err_output_r);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("\tError Output Left:  %f\n", err_output_l);
    printf("\tError Output Right: %f\n", err_output_r);
    printf("\n");
    #endif

    TEST_ASSERT(0.0 > err_output_r);
    TEST_ASSERT(0.0 > err_output_l);
}


void test_PIDrunDistance_PositiveError_WhenUnderTarget(void)
{
    int32_t distance_target = 30;
    float err_output_l = 0;
    float err_output_r = 0;
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_distance);
    motor_calc_PID_set_target(kMotor_Left, &distance_target);
    motor_calc_PID_set_target(kMotor_Right, &distance_target);
    motor_calc_parameters(10, 10, 100);
    motor_calc_PID_run(&err_output_l, &err_output_r);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("\tError Output Left:  %f\n", err_output_l);
    printf("\tError Output Right: %f\n", err_output_r);
    printf("\n");
    #endif

    TEST_ASSERT(0.0 < err_output_r);
    TEST_ASSERT(0.0 < err_output_l);
}


void test_PIDrunDistance_ErrorGrows_IfNoProgress(void)
{
    int32_t distance_target = 100;
    float first_err_output_l = 0;
    float first_err_output_r = 0;
    float second_err_output_l = 0;
    float second_err_output_r = 0;

    // setup initial conditions...
    motor_calc_init(0);
    motor_calc_PID_setmode(kPID_distance);
    motor_calc_PID_set_target(kMotor_Left, &distance_target);
    motor_calc_PID_set_target(kMotor_Right, &distance_target);

    // imitate the first cycle (no movement over 100 ticks)
    motor_calc_parameters(0, 0, 100);
    motor_calc_PID_run(&first_err_output_l, &first_err_output_r);

    // imitate some small movement after 100 more ticks
    motor_calc_parameters(0, 0, 200);
    motor_calc_PID_run(&second_err_output_l, &second_err_output_r);

    // do some output if we've defined verbose output
    #ifdef VERBOSE_OUTPUT
    printf("\nFunction: %s\n", __func__);
    printf("\tDistance Target: %d\tDistance: %d\n", distance_target, motor_calc_distance_get(kMotor_Right));
    printf("\tError Output: %f\n", first_err_output_l);
    printf("\tDistance Target: %d\tDistance: %d\n", distance_target, motor_calc_distance_get(kMotor_Right));
    printf("\tError Output: %f\n", second_err_output_l);
    printf("\n");
    #endif

    // the error output should be slightly more the second time (integral error)
    TEST_ASSERT(first_err_output_l < second_err_output_l);
    TEST_ASSERT(first_err_output_r < second_err_output_r);
}



void test_thisTestFails(void)
{
    TEST_ASSERT(1 == 2);
}


int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_MinMacro);
    RUN_TEST(test_MaxMacro);

    RUN_TEST(test_VelocityCalculation);
    RUN_TEST(test_DistanceCalculation);
    RUN_TEST(test_MotorCalc_NoINF);

    RUN_TEST(test_PIDmodeSet);

    RUN_TEST(test_PIDrunVelocity_ZeroError_WhenTargetMet);
    RUN_TEST(test_PIDrunVelocity_NegativeError_WhenOverTarget);
    RUN_TEST(test_PIDrunVelocity_PositiveError_WhenUnderTarget);
    RUN_TEST(test_PIDrunVelocity_ErrorGrows_IfNoProgress);

    RUN_TEST(test_PIDrunDistance_ZeroError_WhenTargetMet);
    RUN_TEST(test_PIDrunDistance_NegativeError_WhenOverTarget);
    RUN_TEST(test_PIDrunDistance_PositiveError_WhenUnderTarget);
    RUN_TEST(test_PIDrunDistance_ErrorGrows_IfNoProgress);

    RUN_TEST(test_thisTestFails);

    return UNITY_END();
}
