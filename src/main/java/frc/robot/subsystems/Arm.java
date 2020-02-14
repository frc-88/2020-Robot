/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ArmConfig;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */

  private double MAXIMUM_ARM_VELOCITY = 60; //degrees per second

  private double speed;
  private DoublePreferenceConstant MAXIMUM_ARM_ACCELERATION; //degrees / s^2
  private DoublePreferenceConstant rotator_kP;
  private DoublePreferenceConstant rotator_kI;
  private DoublePreferenceConstant rotator_kD;
  private DoublePreferenceConstant rotator_kF;


  private TalonFX m_rotator = new TalonFX(Constants.ARM_MOTOR);

  private ArmConfig armConfig;

  public Arm() {
    m_rotator.configFactoryDefault();
    m_rotator.configAllSettings(armConfig.armConfiguration);

    m_rotator.enableVoltageCompensation(true);
    m_rotator.setInverted(false);
    m_rotator.setSensorPhase(false);
    m_rotator.setNeutralMode(NeutralMode.Brake);

    //Motion magic **woosh**

    MAXIMUM_ARM_ACCELERATION = new DoublePreferenceConstant("CPM Acceleration", 2);
    MAXIMUM_ARM_ACCELERATION.addChangeHandler(
      (Double acceleration) -> m_rotator.configMotionAcceleration(convertArmVelocityToMotorVelocity(acceleration)));
    rotator_kP = new DoublePreferenceConstant("Arm rotator kP", 0);
    rotator_kP.addChangeHandler((Double kP) -> m_rotator.config_kP(0, kP));
    rotator_kI = new DoublePreferenceConstant("Arm rotator kI", 0);
    rotator_kI.addChangeHandler((Double kI) -> m_rotator.config_kI(0, kI));
    rotator_kD = new DoublePreferenceConstant("Arm rotator kD", 0);
    rotator_kD.addChangeHandler((Double kD) -> m_rotator.config_kD(0, kD));
    rotator_kF = new DoublePreferenceConstant("Arm rotator kF", 0);
    rotator_kF.addChangeHandler((Double kF) -> m_rotator.config_kF(0, kF));

    m_rotator.configMotionCruiseVelocity(convertArmVelocityToMotorVelocity(MAXIMUM_ARM_VELOCITY));
    m_rotator.configMotionAcceleration(convertArmVelocityToMotorVelocity(MAXIMUM_ARM_ACCELERATION.getValue()));
  }

  private void moveArmToAngle(double armPosition) {
    m_rotator.set(ControlMode.MotionMagic, convertArmPositionToMotorPosition(armPosition));
  }

  /**
   * Converts motor position to arm angle
   * @param motorPosition, in ticks
   * @return armPosition, in degrees
   */

  private double convertMotorPositionToArmPosition(int motorPosition) {
    return (motorPosition * (1. / Constants.ARM_ENCODER_TICKS_PER_ROTATION) * 360. * (1. / Constants.ARM_ENCODER_RATIO));
  }

  /**
   * Converts arm position to motor position
   * @param armPosition, in degrees 
   * @return motorPosition, in ticks
   */

  private int convertArmPositionToMotorPosition(double armPosition) {
    return (int)(armPosition * (1. / 360.) * Constants.ARM_ENCODER_RATIO * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  }

  /**
   * Converts motor velocity to arm velocity
   * @param motorVelocity, in ticks per 100ms
   * @return armVelocity, in degrees per second
   */

  private double convertMotorVelocityToArmVelocity(int motorVelocity) {
    return (motorVelocity * (1. / 10.) * (1. / Constants.ARM_ENCODER_TICKS_PER_ROTATION) * (1. / Constants.ARM_ENCODER_RATIO) * 360.);
  }

  /**
   * Converts arm velocity to motor velocity
   * @param armVelocity, in degrees per second
   * @return motorVelocity, in ticks per 100ms
   */

  private int convertArmVelocityToMotorVelocity(double armVelocity) {
    return (int)(armVelocity * (1. / 360.) * Constants.ARM_ENCODER_RATIO * 10. * Constants.ARM_ENCODER_TICKS_PER_ROTATION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
