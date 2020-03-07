/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    
    //Climber constants
    public static final int CLIMBER_MOTOR_LEFT = 13;
	public static final int CLIMBER_MOTOR_RIGHT = 2;
	public static final int CLIMBER_PNEUMATICS_FORWARD = 4;
	public static final int CLIMBER_PNEUMATICS_REVERSE = 3;
	public static final int CLIMBER_MIN_POSITION = 0;
	public static final int CLIMBER_MAX_POSITION = (int) (30 * 9 * 2048);
	public static final int CLIMBER_EXPONENTIAL = 2;
	public static final double CLIMBER_CONTROLLER_DEADZONE = 0.12;
	
	// Shooter constants
	public static final int SHOOTER_FLYWHEEL_MASTER = 12;
	public static final int SHOOTER_FLYWHEEL_FOLLOWER = 3;
	public static final double SHOOTER_FLYWHEEL_MAX_SPEED = 1;
	public static final double SHOOTER_FLYWHEEL_TOLERANCE = 100;
	public static final int SHOOTER_FLYWHEEL_SPEED_EXP = 1;
	public static final double SHOOTER_MOTOR_TO_FLYWHEEL_RATIO = 1.5;
	public static final int SHOOTER_MOTOR_TICKS_PER_ROTATION = 2048;

	// Feeder constants
	public static final int FEEDER_MOTOR = 8;
	public static final int FEEDER_CPM_PCM = 1;
	public static final int FEEDER_CPM_PISTON_FORWARD = 0;
	public static final int FEEDER_CPM_PISTON_REVERSE = 7;

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
	public static final double ARM_LOW_LIMIT = -5;
	public static final double ARM_HIGH_LIMIT = 93;
	public static final double ARM_ENCODER_SHIFT = 15;
	public static final int ARM_SPEED_EXP = 2;
	public static final double ARM_TOLERANCE = 5;
	public static final int ARM_COAST_DIO = 9;

	// Intake constants
	public static final int ROLLER_ID = 11;
	public static final int INTAKE_DEPLOY_PISTON = 0;
	public static final int INTAKE_RETRACT_PISTON = 7;

	// Sensor constants
	public static final String PCC_CAMERA_NAME = "PCC";
	public static final String PCC_STREAM_NAME = "PCC";
	public static final int PCC_CAMERA_ID = 1;
	public static final int PCC_IMAGE_WIDTH = 640;
	public static final int PCC_IMAGE_HEIGHT = 480;
	public static final int PCC_BLUR = 10;
	public static final int PCC_HUE_LO = 10;
	public static final int PCC_HUE_HI = 80;
	public static final int PCC_SAT_LO = 20;
	public static final int PCC_SAT_HI = 255;
	public static final int PCC_VAL_LO = 20;
	public static final int PCC_VAL_HI = 255;

	public static final int PCC_CHAMBER_X = 200;
	public static final int PCC_CHAMBER_Y = 285;
	public static final int PCC_CHAMBER_WIDTH = 185;
	public static final int PCC_CHAMBER_HEIGHT = 185;
	public static final double PCC_CHAMBER_THRESHOLD = 30000;

	// Hopper constants
	public static final int LEFT_HOPPER = 9;
	public static final int RIGHT_HOPPER = 10;
	public static final double HOPPER_SHOOT_PERCENT_OUTPUT = .5;
	public static final double HOPPER_INTAKE_PERCENT_OUTPUT = 0.5;

	// Sensor constants
	public static final int SHOOTER_BALL_SENSOR_ID = 1;

	// Field constants
	public static final double FIELD_PORT_TARGET_HEIGHT = 90.75;
	public static final double FIELD_INNER_PORT_OFFSET = 29.25;

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
	public static final int LEFT_MASTER_DRIVE_ID = 0;
	public static final int LEFT_FOLLOWER_DRIVE_ID = 1;
	public static final int RIGHT_MASTER_DRIVE_ID = 15;
	public static final int RIGHT_FOLLOWER_DRIVE_ID = 14;
	public static final int LEFT_DRIVE_ENCODER_ID = 0;
	public static final int RIGHT_DRIVE_ENCODER_ID = 15;
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
	public static final double CHEESY_DRIVE_FORCE_LOW_MIN_TURN = 0.6;
	public static final double CHEESY_DRIVE_FORCE_LOW_MAX_TURN = 1.;

	//Test
	public static final double TEST_JOYSTICK_DEADBAND = 0.12;

	/*******************************************************************/

}
