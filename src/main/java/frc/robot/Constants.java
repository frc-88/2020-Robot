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
	public static final int SHOOTER_MOTOR = 07;
	public static final int SHOOTER_MOTOR_2 = 8;
	public static final int SHOOTER_ANGLE_ENCODER_CHANNEL_1A = 9;
	public static final int SHOOTER_ANGLE_ENCODER_CHANNEL_1B = 10;
	public static final int SHOOTER_FEEDER_MOTOR = 0;
	public static final int SHOOTER_ROTATOR_MOTOR = 0;

	// Intake constants
	public static final int ROLLER_ID = 11;
	public static final int INTAKE_LEFT_DEPLOY_PISTON = 1;
	public static final int INTAKE_LEFT_RETRACT_PISTON = 6;
	public static final int INTAKE_RIGHT_DEPLOY_PISTON = 0;
	public static final int INTAKE_RIGHT_RETRACT_PISTON = 7;


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
	public static final int SHIFTER_LEFT_OUT = 1;
	public static final int SHIFTER_LEFT_IN = 6;
	public static final int SHIFTER_RIGHT_PCM = 0;
	public static final int SHIFTER_RIGHT_OUT = 0;
	public static final int SHIFTER_RIGHT_IN = 7;

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

	/*******************************************************************/

}
