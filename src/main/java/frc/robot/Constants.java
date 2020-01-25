/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int CPMMotor = 00;
    public static final int CPMJointEncoderChannelA = 01;
    public static final int CPMJointEncoderChannelB = 02;
    public static final int CPMPneumaticsForward = 03;
    public static final int CPMPneumaticsReverse = 04;
    public static final int CPMDigitalInputChannel = 05;
    public static final int CPMWristEncoderCountsPerRev = 06;

    public static final int ROLLER_ID = 5;
    public static final int INTAKE_DEPLOY_PISTON = 0;
    public static final int INTAKE_RETRACT_PISTON = 7;
    




    /********************************************************************
    * 
    *     ____       _          
    *    / __ \_____(_)   _____ 
    *   / / / / ___/ / | / / _ \
    *  / /_/ / /  / /| |/ /  __/
    * /_____/_/  /_/ |___/\___/ 
    * 
    */
                            
    // Drive CAN IDs
    public static final int LEFT_MASTER_DRIVE_ID = 0;
	public static final int LEFT_TALON_FOLLOWER_DRIVE_ID = 0;
	public static final int RIGHT_MASTER_DRIVE_ID = 0;
	public static final int RIGHT_TALON_FOLLOWER_DRIVE_ID = 0;
	public static final int SHIFTER_LEFT_PCM = 0;
	public static final int SHIFTER_LEFT_OUT = 0;
	public static final int SHIFTER_LEFT_IN = 0;
	public static final int SHIFTER_RIGHT_PCM = 0;
	public static final int SHIFTER_RIGHT_OUT = 0;
	public static final int SHIFTER_RIGHT_IN = 0;

    // Drive Configuration 
    public static final int NUM_DRIVE_MOTORS_PER_SIDE = 2;
	public static final double LOW_DRIVE_RATIO = 0;
	public static final double HIGH_DRIVE_RATIO = 0;
	public static final double DRIVE_SENSOR_RATIO = 0;
	public static final double DRIVE_LOW_STATIC_FRICTION_VOLTAGE = 0;
	public static final double DRIVE_HIGH_STATIC_FRICTION_VOLTAGE = 0;
	public static final double DRIVE_LEFT_LOW_EFFICIENCY = 0;
	public static final double DRIVE_LEFT_HIGH_EFFICIENCY = 0;
	public static final double DRIVE_RIGHT_LOW_EFFICIENCY = 0;
	public static final double DRIVE_RIGHT_HIGH_EFFICIENCY = 0;
	public static final double MAX_SPEED_LOW = 0;
	public static final double MAX_SPEED_HIGH = 0;
	public static final double DRIVE_CURRENT_LIMIT = 0;
	public static final int DRIVE_SPEED_EXP = 0;
	public static final int DRIVE_TURN_EXP = 0;
	public static final double MAX_ACCEL_HIGH_TIPPY = 0;
	public static final double MAX_ACCEL_HIGH = 0;
	public static final double MAX_ACCEL_LOW_TIPPY = 0;
	public static final double MAX_ACCEL_LOW = 0;
}

