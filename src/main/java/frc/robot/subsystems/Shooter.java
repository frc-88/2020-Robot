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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ShooterConfig;

public class Shooter extends SubsystemBase {
  private TalonFX m_flywheelMaster = new TalonFX(Constants.SHOOTER_FLYWHEEL_MASTER);
  private TalonFX m_flywheelFollower = new TalonFX(Constants.SHOOTER_FLYWHEEL_FOLLOWER);
  private TalonSRX m_feeder = new TalonSRX(Constants.SHOOTER_FEEDER_MOTOR);
  private ShooterConfig m_shooterConfig = new ShooterConfig();

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_flywheelMaster.configFactoryDefault();
    m_flywheelMaster.configAllSettings(m_shooterConfig.flywheelConfiguration);
    m_flywheelMaster.enableVoltageCompensation(true);
    m_flywheelMaster.setInverted(false);
    m_flywheelMaster.setSensorPhase(false);
    m_flywheelMaster.setNeutralMode(NeutralMode.Coast);

    m_flywheelFollower.configFactoryDefault();
    m_flywheelFollower.enableVoltageCompensation(true);
    m_flywheelFollower.setInverted(false);
    m_flywheelFollower.setSensorPhase(false);
    m_flywheelFollower.setNeutralMode(NeutralMode.Coast);
    m_flywheelFollower.follow(m_flywheelMaster);

    m_feeder.configFactoryDefault();
    m_feeder.enableVoltageCompensation(true);
    m_feeder.setInverted(false);
    m_feeder.setSensorPhase(false);
    m_feeder.setNeutralMode(NeutralMode.Brake);
  }

  public void setFlywheel(double velocity) {
    m_flywheelMaster.set(ControlMode.Velocity, velocity * Constants.SHOOTER_FLYWHEEL_MAX_SPEED);
  }

  public void setFlywheelBasic(double percentOutput) {
    m_flywheelMaster.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setFeeder(double percentOutput) {
    m_feeder.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
