/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	// CPM constants
	public static final int CPM_MOTOR = 00;
	public static final int CPM_JOINT_ENCODER_CHANNEL_1A = 01;
	public static final int CPM_JOINT_ENCODER_CHANNEL_1B = 02;
	public static final int CPM_PNEUMATICS_FORWARD = 03;
	public static final int CPM_PNEUMATICS_REVERSE = 04;
	public static final int CPM_DIGITAL_INPUT_CHANNEL = 00;
	public static final int CPM_WRIST_ENCODER_COUNTS_PER_REV = 06;
	public static final float CPM_PHASE_2_WHEEL_ROTATIONS = 4;

	// Shooter constants
	public static final int SHOOTER_FLYWHEEL_MASTER = 12;
	public static final int SHOOTER_FLYWHEEL_FOLLOWER = 3;
	public static final double SHOOTER_FLYWHEEL_MAX_SPEED = 1;
	public static final double SHOOTER_FLYWHEEL_TOLERANCE = 25;
	public static final int SHOOTER_FLYWHEEL_SPEED_EXP = 1;
	public static final double SHOOTER_MOTOR_TO_FLYWHEEL_RATIO = 1.5;
	public static final int SHOOTER_MOTOR_TICKS_PER_ROTATION = 2048;

	// Feeder constants
	public static final int FEEDER_MOTOR = 8;

	// Arm constants
	public static final int ARM_MOTOR = 4;
	public static final int ARM_CANCODER = 4;
	public static final double ARM_SHOOTER_ANGLE_1 = 0;
	public static final double ARM_SHOOTER_ANGLE_2 = 45;
	public static final double ARM_SHOOTER_ANGLE_3 = 90;
	public static final int ARM_ENCODER_TICKS_PER_ROTATION = 4096;
	public static final int FALCON_TO_ENCODER_RATIO = 2;
	public static final int ENCODER_TO_ARM_RATIO = 3;
	public static final int FALCON_TO_ARM_RATIO = 6;
	public static final double ARM_MAXIMUM_VELOCITY = 60;
	public static final double ARM_LOW_LIMIT = 0;
	public static final double ARM_HIGH_LIMIT = 90;
	public static final double ARM_ENCODER_SHIFT = 15;
	public static final int ARM_SPEED_EXP = 2;
	public static final double ARM_TOLERANCE = 0.5;

	// Intake constants
	public static final int ROLLER_ID = 11;
	public static final int INTAKE_DEPLOY_PISTON = 0;
	public static final int INTAKE_RETRACT_PISTON = 7;

	// Hopper constants
	public static final int LEFT_HOPPER = 9;
	public static final int RIGHT_HOPPER = 10;
	public static final double HOPPER_SHOOT_PERCENT_OUTPUT = .5;
	public static final double HOPPER_INTAKE_PERCENT_OUTPUT = 0.5;

	// Sensor constants
	public static final double LIMELIGHT_HEIGHT = 21.0;
	public static final double LIMELIGHT_ANGLE = 5.0; // TODO
	public static final double LIMELIGHT_CENTER_OFFSET = 8.0;

	// Field constants
	public static final double FIELD_PORT_TARGET_HEIGHT = 90.75;

	/********************************************************************
    * 
    *                ____       _          
    *       --------/ __ \_____(_)   _____ 
    *      --------/ / / / ___/ / | / / _ \
    *     --------/ /_/ / /  / /| |/ /  __/
    *    --------/_____/_/  /_/ |___/\___/ 
    * 
	*/
	
	// Drive CAN IDs
	public static final int LEFT_MASTER_DRIVE_ID = 15;
	public static final int LEFT_FOLLOWER_DRIVE_ID = 14;
	public static final int RIGHT_MASTER_DRIVE_ID = 0;
	public static final int RIGHT_FOLLOWER_DRIVE_ID = 1;
	public static final int LEFT_DRIVE_ENCODER_ID = 15;
	public static final int RIGHT_DRIVE_ENCODER_ID = 0;
	public static final int SHIFTER_LEFT_PCM = 0;
	public static final int SHIFTER_LEFT_OUT = 2;
	public static final int SHIFTER_LEFT_IN = 5;
	public static final int SHIFTER_RIGHT_PCM = 0;
	public static final int SHIFTER_RIGHT_OUT = 1;
	public static final int SHIFTER_RIGHT_IN = 6;

	// Drive Configuration
	public static final int NUM_DRIVE_MOTORS_PER_SIDE = 2;
	public static final double LOW_DRIVE_RATIO = (1. / 18.38) * (5.99/12.) * Math.PI;
	public static final double HIGH_DRIVE_RATIO = (1. / 8.50) * (5.99/12.) * Math.PI;;
	public static final double DRIVE_SENSOR_RATIO = (1. / ((5.99/12.) * 3.14159)) * 7.5;
	public static final double DRIVE_LOW_STATIC_FRICTION_VOLTAGE = 0.2;
	public static final double DRIVE_HIGH_STATIC_FRICTION_VOLTAGE = 0.24;
	public static final double DRIVE_LEFT_LOW_EFFICIENCY = 1.025;
	public static final double DRIVE_LEFT_HIGH_EFFICIENCY = 1.03;
	public static final double DRIVE_RIGHT_LOW_EFFICIENCY = 1.03;
	public static final double DRIVE_RIGHT_HIGH_EFFICIENCY = 1.02;
	public static final double MAX_SPEED_LOW = 8.8;
	public static final double MAX_SPEED_HIGH = 18;
	public static final double WHEEL_BASE_WIDTH = (25. + 5./16.) / 12.; // feet
	public static final double DRIVE_CURRENT_LIMIT = 250;
	public static final int DRIVE_SPEED_EXP = 2;
	public static final int DRIVE_TURN_EXP = 2;
	public static final double DRIVE_JOYSTICK_DEADBAND = 0.12;
	public static final double CHEESY_DRIVE_MIN_TURN = 0.4;
	public static final double CHEESY_DRIVE_MAX_TURN = 0.6;

	//Test
	public static final double TEST_JOYSTICK_DEADBAND = 0.12;

	/*******************************************************************/

}
