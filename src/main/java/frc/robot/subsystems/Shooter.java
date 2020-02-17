/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ShooterConfig;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Shooter extends SubsystemBase {
  private TalonFX m_flywheelMaster = new TalonFX(Constants.SHOOTER_FLYWHEEL_MASTER);
  private TalonFX m_flywheelFollower = new TalonFX(Constants.SHOOTER_FLYWHEEL_FOLLOWER);
  private TalonSRX m_feeder = new TalonSRX(Constants.SHOOTER_FEEDER_MOTOR);
  private ShooterConfig m_shooterConfig = new ShooterConfig();

  private DoublePreferenceConstant flywheel_kP;
  private DoublePreferenceConstant flywheel_kI;
  private DoublePreferenceConstant flywheel_kD;
  private DoublePreferenceConstant flywheel_kF;
  private DoublePreferenceConstant flywheel_iZone;
  private DoublePreferenceConstant flywheel_iMax;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_flywheelMaster.configFactoryDefault();
    m_flywheelMaster.configAllSettings(m_shooterConfig.flywheelConfiguration);
    m_flywheelMaster.enableVoltageCompensation(true);
    m_flywheelMaster.setInverted(InvertType.InvertMotorOutput);
    m_flywheelMaster.setSensorPhase(false);
    m_flywheelMaster.setNeutralMode(NeutralMode.Coast);

    m_flywheelFollower.configFactoryDefault();
    m_flywheelFollower.enableVoltageCompensation(true);
    m_flywheelFollower.setInverted(InvertType.OpposeMaster);
    m_flywheelFollower.setSensorPhase(false);
    m_flywheelFollower.setNeutralMode(NeutralMode.Coast);
    m_flywheelFollower.follow(m_flywheelMaster);

    m_feeder.configFactoryDefault();
    m_feeder.enableVoltageCompensation(true);
    m_feeder.setInverted(false);
    m_feeder.setSensorPhase(false);
    m_feeder.setNeutralMode(NeutralMode.Brake);

    flywheel_kP = new DoublePreferenceConstant("Shooter flywheel kP", 0);
    flywheel_kP.addChangeHandler((Double kP) -> m_flywheelMaster.config_kP(0, kP));
    m_flywheelMaster.config_kP(0, flywheel_kP.getValue());
    flywheel_kI = new DoublePreferenceConstant("Shooter flywheel kI", 0);
    flywheel_kI.addChangeHandler((Double kI) -> m_flywheelMaster.config_kI(0, kI));
    m_flywheelMaster.config_kI(0, flywheel_kI.getValue());
    flywheel_kD = new DoublePreferenceConstant("Shooter flywheel kD", 0);
    flywheel_kD.addChangeHandler((Double kD) -> m_flywheelMaster.config_kD(0, kD));
    m_flywheelMaster.config_kD(0, flywheel_kD.getValue());
    flywheel_kF = new DoublePreferenceConstant("Shooter flywheel kF", 0);
    flywheel_kF.addChangeHandler((Double kF) -> m_flywheelMaster.config_kF(0, kF));
    m_flywheelMaster.config_kF(0, flywheel_kF.getValue());
    flywheel_iZone = new DoublePreferenceConstant("Shooter flywheel iZone", 0);
    flywheel_iZone.addChangeHandler((Double iZone) -> m_flywheelMaster.config_IntegralZone(0, convertFlywheelVelocityToEncoderVelocity(iZone)));
    m_flywheelMaster.config_IntegralZone(0, convertFlywheelVelocityToEncoderVelocity(flywheel_iZone.getValue()));
    flywheel_iMax = new DoublePreferenceConstant("Shooter flywheel iMax", 0);
    flywheel_iMax.addChangeHandler((Double iMax) -> m_flywheelMaster.configMaxIntegralAccumulator(0, iMax));
    m_flywheelMaster.configMaxIntegralAccumulator(0, convertFlywheelVelocityToEncoderVelocity(flywheel_iMax.getValue()));


  }

  public void setFlywheel(double velocity) {
    m_flywheelMaster.set(ControlMode.Velocity, convertFlywheelVelocityToEncoderVelocity(velocity));
  }

  public boolean flywheelOnTarget() {
    return Math.abs(convertEncoderVelocityToFlywheelVelocity(m_flywheelMaster.getClosedLoopError())) < Constants.SHOOTER_FLYWHEEL_TOLERANCE;
  }

  public void setFlywheelBasic(double percentOutput) {
    m_flywheelMaster.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setFeeder(double percentOutput) {
    m_feeder.set(ControlMode.PercentOutput, percentOutput);
  }

  public double convertEncoderVelocityToFlywheelVelocity(int ticks) {
    return (ticks * 10. * 60. * (1. / Constants.SHOOTER_MOTOR_TICKS_PER_ROTATION) * Constants.SHOOTER_MOTOR_TO_FLYWHEEL_RATIO);
  }

  public int convertFlywheelVelocityToEncoderVelocity(double rpm) {
    return (int)(rpm * (1. / 10.) * (1. / 60.) * Constants.SHOOTER_MOTOR_TICKS_PER_ROTATION * (1. / Constants.SHOOTER_MOTOR_TO_FLYWHEEL_RATIO));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel velocity", convertEncoderVelocityToFlywheelVelocity(m_flywheelMaster.getSelectedSensorVelocity()));
    // This method will be called once per scheduler run
  }
}
