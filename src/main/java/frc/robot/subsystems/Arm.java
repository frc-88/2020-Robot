/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.VelocityPeriod;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ArmConfig;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */

  private DoublePreferenceConstant ARM_OFFSET;

  private DoublePreferenceConstant MAXIMUM_ARM_VELOCITY; //degrees per second
  private DoublePreferenceConstant MAXIMUM_ARM_ACCELERATION;

  private DoublePreferenceConstant rotator_kP;
  private DoublePreferenceConstant rotator_kI;
  private DoublePreferenceConstant rotator_kD;
  private DoublePreferenceConstant rotator_kF;

  private TalonFX m_rotator = new TalonFX(Constants.ARM_MOTOR);
  private CANCoder m_armEncoder = new CANCoder(Constants.ARM_CANCODER);

  private ArmConfig armConfig = new ArmConfig();

  private int remoteSensorID = 0;

  private int armOffsetTicks = 0;

  public Arm() {
    m_armEncoder.configFactoryDefault();
    m_armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    m_armEncoder.configSensorDirection(true);
    /* Configure velocity measurements to be what we want */
		// m_armEncoder.configVelocityMeasurementPeriod(VelocityPeriod.Period_100Ms);
    // m_armEncoder.configVelocityMeasurementWindow(64);

    m_rotator.configFactoryDefault();
    m_rotator.configAllSettings(armConfig.armConfiguration);

    m_rotator.enableVoltageCompensation(true);
    m_rotator.setInverted(false);
    m_rotator.setSensorPhase(false);
    m_rotator.setNeutralMode(NeutralMode.Brake);

    ARM_OFFSET = new DoublePreferenceConstant("Arm Offset", 0);

    //Motion magic **woosh**

    MAXIMUM_ARM_VELOCITY = new DoublePreferenceConstant("Arm velocity", 180);
    MAXIMUM_ARM_VELOCITY.addChangeHandler(
      (Double velocity) -> m_rotator.configMotionCruiseVelocity(convertArmVelocityToEncoderVelocity(velocity)));
    MAXIMUM_ARM_ACCELERATION = new DoublePreferenceConstant("Arm Acceleration", 220);
    MAXIMUM_ARM_ACCELERATION.addChangeHandler(
      (Double acceleration) -> m_rotator.configMotionAcceleration(convertArmVelocityToEncoderVelocity(acceleration)));
    rotator_kP = new DoublePreferenceConstant("Arm rotator kP", 1);
    rotator_kP.addChangeHandler((Double kP) -> m_rotator.config_kP(0, kP));
    rotator_kI = new DoublePreferenceConstant("Arm rotator kI", 0.0003);
    rotator_kI.addChangeHandler((Double kI) -> m_rotator.config_kI(0, kI));
    rotator_kD = new DoublePreferenceConstant("Arm rotator kD", 0.1);
    rotator_kD.addChangeHandler((Double kD) -> m_rotator.config_kD(0, kD));
    rotator_kF = new DoublePreferenceConstant("Arm rotator kF", 2.2);
    rotator_kF.addChangeHandler((Double kF) -> m_rotator.config_kF(0, kF));

    m_rotator.configMotionCruiseVelocity(convertArmVelocityToEncoderVelocity(MAXIMUM_ARM_VELOCITY.getValue()));
    m_rotator.configMotionAcceleration(convertArmVelocityToEncoderVelocity(MAXIMUM_ARM_ACCELERATION.getValue()));

    m_rotator.configRemoteFeedbackFilter(m_armEncoder.getDeviceID(), RemoteSensorSource.CANCoder, remoteSensorID);

    zeroArm();
  }

  public void setArmPosition(double armPosition) {
    m_rotator.set(ControlMode.MotionMagic, convertArmDegreesToEncoderTicks(armPosition));
  }

  public void setPercentOutput(double percent) {
    m_rotator.set(ControlMode.PercentOutput, percent);
  }
  
  public void zeroArm() {
    double angle = (m_armEncoder.getAbsolutePosition() / Constants.ENCODER_TO_ARM_RATIO) + ARM_OFFSET.getValue();
    angle = (angle + Constants.ARM_ENCODER_SHIFT) % (360. / Constants.ENCODER_TO_ARM_RATIO);
    if(angle < 0) {
      angle += 360. / Constants.ENCODER_TO_ARM_RATIO;
    }
    angle -= Constants.ARM_ENCODER_SHIFT;
    armOffsetTicks = convertArmDegreesToEncoderTicks(angle) - m_rotator.getSelectedSensorPosition();
    m_rotator.configForwardSoftLimitEnable(true);
    m_rotator.configForwardSoftLimitThreshold(convertArmDegreesToEncoderTicks(Constants.ARM_HIGH_LIMIT));
    m_rotator.configReverseSoftLimitEnable(true);
    m_rotator.configReverseSoftLimitThreshold(convertArmDegreesToEncoderTicks(Constants.ARM_LOW_LIMIT));
  }

  public void calibrateArm() {
    ARM_OFFSET.setValue(90 - (m_armEncoder.getAbsolutePosition() / Constants.ENCODER_TO_ARM_RATIO));
    zeroArm();
  }

  public void setToBrakeMode() {
    m_rotator.setNeutralMode(NeutralMode.Brake);
  }

  public double getCurrentArmPosition() {
    return convertEncoderTicksToArmDegrees(m_rotator.getSelectedSensorPosition());
  }

  /**
   * Converts encoder position to arm position
   * @param encoderPosition, in ticks
   * @return armPosition, in degrees
   */

  public double convertEncoderTicksToArmDegrees(int encoderPosition) {
    encoderPosition += armOffsetTicks;
    return (encoderPosition * (1. / Constants.ARM_ENCODER_TICKS_PER_ROTATION) * 360. * (1. / Constants.ENCODER_TO_ARM_RATIO));
  }

  /**
   * Converts arm position to motor position
   * @param armPosition, in degrees 
   * @return motorPosition, in ticks
   */

  public int convertArmDegreesToEncoderTicks(double armPosition) {
    return (int)(armPosition * (1. / 360.) * Constants.ENCODER_TO_ARM_RATIO * Constants.ARM_ENCODER_TICKS_PER_ROTATION) - armOffsetTicks;
  }

  /**
   * Converts motor velocity to arm velocity
   * @param motorVelocity, in ticks per 100ms
   * @return armVelocity, in degrees per second
   */

  public double convertEncoderVelocityToArmVelocity(int encoderVelocity) {
    return (encoderVelocity * 10. * (1. / Constants.ARM_ENCODER_TICKS_PER_ROTATION) * (1. / Constants.ENCODER_TO_ARM_RATIO) * 360.);
  }

  /**
   * Converts arm velocity to motor velocity
   * @param armVelocity, in degrees per second
   * @return motorVelocity, in ticks per 100ms
   */

  public int convertArmVelocityToEncoderVelocity(double armVelocity) {
    return (int)(armVelocity * (1. / 360.) * Constants.ENCODER_TO_ARM_RATIO * (1. / 10.) * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  }

  // public int convertArmDegreesToMotorTicks(double armPosition) {
  //   return (int)(armPosition * (1. / 360.) * Constants.FALCON_TO_ARM_RATIO * 2. * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  // }

  // public int convertArmVelocityToMotorVelocity(double armVelocity) {
  //   return (int)(armVelocity * (1. / 360.) * Constants.FALCON_TO_ARM_RATIO * 10. * Constants.FALCON_TO_ENCODER_RATIO * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", convertEncoderTicksToArmDegrees(m_rotator.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Arm Velocity", convertEncoderVelocityToArmVelocity(m_rotator.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Expected Arm Position", convertEncoderTicksToArmDegrees(m_rotator.getActiveTrajectoryPosition()));
    SmartDashboard.putNumber("Expected Arm Velocity", convertEncoderVelocityToArmVelocity(m_rotator.getActiveTrajectoryVelocity()));

    SmartDashboard.putNumber("Arm Current Use", m_rotator.getSupplyCurrent());
    SmartDashboard.putNumber("Arm Abs Encoder Pos", m_armEncoder.getAbsolutePosition() / Constants.ENCODER_TO_ARM_RATIO);
  }
}
