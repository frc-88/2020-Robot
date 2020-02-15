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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ArmConfig;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */

  private double MAXIMUM_ARM_VELOCITY = 60; //degrees per second
  private DoublePreferenceConstant MAXIMUM_ARM_ACCELERATION;

  private DoublePreferenceConstant rotator_kP;
  private DoublePreferenceConstant rotator_kI;
  private DoublePreferenceConstant rotator_kD;
  private DoublePreferenceConstant rotator_kF;

  private TalonFX m_rotator = new TalonFX(Constants.ARM_MOTOR);
  private CANifier m_armEncoder = new CANifier(Constants.ARM_CANIFIER);

  private ArmConfig armConfig;

  private int remoteSensorID = 0;

  public Arm() {
    m_armEncoder.configFactoryDefault();
    /* Configure velocity measurements to be what we want */
		m_armEncoder.configVelocityMeasurementPeriod(VelocityPeriod.Period_100Ms);
    m_armEncoder.configVelocityMeasurementWindow(64);

    m_rotator.configFactoryDefault();
    m_rotator.configAllSettings(armConfig.armConfiguration);

    m_rotator.enableVoltageCompensation(true);
    m_rotator.setInverted(false);
    m_rotator.setSensorPhase(false);
    m_rotator.setNeutralMode(NeutralMode.Brake);

    //Motion magic **woosh**

    MAXIMUM_ARM_ACCELERATION = new DoublePreferenceConstant("Arm Acceleration", 15);
    MAXIMUM_ARM_ACCELERATION.addChangeHandler(
      (Double acceleration) -> m_rotator.configMotionAcceleration(convertArmVelocityToEncoderVelocity(acceleration)));
    rotator_kP = new DoublePreferenceConstant("Arm rotator kP", 0);
    rotator_kP.addChangeHandler((Double kP) -> m_rotator.config_kP(0, kP));
    rotator_kI = new DoublePreferenceConstant("Arm rotator kI", 0);
    rotator_kI.addChangeHandler((Double kI) -> m_rotator.config_kI(0, kI));
    rotator_kD = new DoublePreferenceConstant("Arm rotator kD", 0);
    rotator_kD.addChangeHandler((Double kD) -> m_rotator.config_kD(0, kD));
    rotator_kF = new DoublePreferenceConstant("Arm rotator kF", 0);
    rotator_kF.addChangeHandler((Double kF) -> m_rotator.config_kF(0, kF));

    m_rotator.configMotionCruiseVelocity(convertArmVelocityToEncoderVelocity(MAXIMUM_ARM_VELOCITY));
    m_rotator.configMotionAcceleration(convertArmVelocityToEncoderVelocity(MAXIMUM_ARM_ACCELERATION.getValue()));

    m_rotator.configRemoteFeedbackFilter(m_armEncoder.getDeviceID(), RemoteSensorSource.CANifier_PWMInput0, remoteSensorID);
    m_rotator.configForwardSoftLimitEnable(true);
    m_rotator.configForwardSoftLimitThreshold(Constants.CANIFIER_HIGH_LIMIT);
    m_rotator.configReverseSoftLimitEnable(true);
    m_rotator.configReverseSoftLimitThreshold(Constants.CANIFIER_LOW_LIMIT);
  }

  private void setArmPosition(double armPosition) {
    m_rotator.set(ControlMode.MotionMagic, convertArmDegreesToEncoderTicks(armPosition));
  }

  private void setBasicPosition(double armPosition) {
    m_rotator.set(ControlMode.PercentOutput, convertArmDegreesToEncoderTicks(armPosition));
  }
  
  private double getArmPosition() {
    return ((m_armEncoder.getQuadraturePosition() + Constants.ARM_ENCODER_SHIFT) % (360. / Constants.ENCODER_TO_ARM_RATIO)) - Constants.ARM_ENCODER_SHIFT;
  }

  /**
   * Converts encoder position to arm position
   * @param encoderPosition, in ticks
   * @return armPosition, in degrees
   */

  private double convertEncoderTicksToArmDegrees(int encoderPosition) {
    return (encoderPosition * (1. / Constants.ARM_ENCODER_TICKS_PER_ROTATION) * 360. * (1. / Constants.ENCODER_TO_ARM_RATIO));
  }

  /**
   * Converts arm position to motor position
   * @param armPosition, in degrees 
   * @return motorPosition, in ticks
   */

  private int convertArmDegreesToEncoderTicks(double armPosition) {
    return (int)(armPosition * (1. / 360.) * Constants.ENCODER_TO_ARM_RATIO * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  }

  /**
   * Converts motor velocity to arm velocity
   * @param motorVelocity, in ticks per 100ms
   * @return armVelocity, in degrees per second
   */

  private double convertEncoderVelocityToArmVelocity(int encoderVelocity) {
    return (encoderVelocity * (1. / 10.) * (1. / Constants.ARM_ENCODER_TICKS_PER_ROTATION) * (1. / Constants.ENCODER_TO_ARM_RATIO) * 360.);
  }

  /**
   * Converts arm velocity to motor velocity
   * @param armVelocity, in degrees per second
   * @return motorVelocity, in ticks per 100ms
   */

  private int convertArmVelocityToEncoderVelocity(double armVelocity) {
    return (int)(armVelocity * (1. / 360.) * Constants.ENCODER_TO_ARM_RATIO * 10. * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  }

  private int convertArmDegreesToMotorTicks(double armPosition) {
    return (int)(armPosition * (1. / 360.) * Constants.FALCON_TO_ARM_RATIO * 2. * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  }

  private int convertArmVelocityToMotorVelocity(double armVelocity) {
    return (int)(armVelocity * (1. / 360.) * Constants.FALCON_TO_ARM_RATIO * 10. * Constants.FALCON_TO_ENCODER_RATIO * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Falcon Position", m_rotator.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Canifier Position", m_armEncoder.getQuadraturePosition());
  }
}
