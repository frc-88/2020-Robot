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
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ShooterConfig;
import frc.robot.util.ValueInterpolator;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Shooter extends SubsystemBase {
  private Sensors m_sensors;
  private int m_loopCounter = 0;

  private TalonFX m_flywheelMaster = new TalonFX(Constants.SHOOTER_FLYWHEEL_MASTER);
  private TalonFX m_flywheelFollower = new TalonFX(Constants.SHOOTER_FLYWHEEL_FOLLOWER);
  private ShooterConfig m_shooterConfig = new ShooterConfig();

  private double lastSpeed = 5200;

  private DoublePreferenceConstant flywheel_kP;
  private DoublePreferenceConstant flywheel_kI;
  private DoublePreferenceConstant flywheel_kD;
  private DoublePreferenceConstant flywheel_kF;
  private DoublePreferenceConstant flywheel_iZone;
  private DoublePreferenceConstant flywheel_iMax;

  private final ValueInterpolator distanceToSpeedInterpolator = new ValueInterpolator(
    new ValueInterpolator.ValuePair(92, 5375),
    new ValueInterpolator.ValuePair(113, 5280),
    new ValueInterpolator.ValuePair(119, 5280),
    new ValueInterpolator.ValuePair(128, 5280),
    new ValueInterpolator.ValuePair(162, 5260),
    new ValueInterpolator.ValuePair(224, 5360),
    new ValueInterpolator.ValuePair(239, 5390),
    new ValueInterpolator.ValuePair(254, 5490),
    new ValueInterpolator.ValuePair(277, 5390),
    new ValueInterpolator.ValuePair(284, 5580)
  );

  /**
   * Creates a new Shooter.
   */
  public Shooter(Sensors sensors) {

    m_sensors = sensors;

    m_flywheelMaster.configFactoryDefault();
    m_flywheelMaster.configAllSettings(m_shooterConfig.flywheelConfiguration);
    m_flywheelMaster.enableVoltageCompensation(true);
    m_flywheelMaster.setInverted(InvertType.InvertMotorOutput);
    m_flywheelMaster.setSensorPhase(false);
    m_flywheelMaster.setNeutralMode(NeutralMode.Coast);

    StatorCurrentLimitConfiguration currentLimit = new StatorCurrentLimitConfiguration();
    currentLimit.enable = true;
    currentLimit.triggerThresholdTime = 0.001;
    currentLimit.triggerThresholdCurrent = 80;
    currentLimit.currentLimit = 80;
    m_flywheelMaster.configStatorCurrentLimit(currentLimit);

    m_flywheelFollower.configFactoryDefault();
    m_flywheelFollower.setInverted(InvertType.OpposeMaster);
    m_flywheelFollower.setSensorPhase(false);
    m_flywheelFollower.setNeutralMode(NeutralMode.Coast);
    m_flywheelFollower.follow(m_flywheelMaster);

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
    if (velocity > 2000) {
      lastSpeed = velocity;
    }
  } 

  public void setFlywheelFromLimelight() {
    if (m_sensors.doesLimelightHaveTarget()) {
      this.lastSpeed = Math.max(distanceToSpeedInterpolator.getInterpolatedValue(m_sensors.getDistanceToTarget()), 5200);
      this.setFlywheel(lastSpeed);
    } else {
      this.setFlywheel(lastSpeed);
    }
  }

  public boolean flywheelOnTarget() {
    return Math.abs(lastSpeed - convertEncoderVelocityToFlywheelVelocity(m_flywheelMaster.getSelectedSensorVelocity())) <= Constants.SHOOTER_FLYWHEEL_TOLERANCE;
  }

  public void setFlywheelBasic(double percentOutput) {
    m_flywheelMaster.set(ControlMode.PercentOutput, percentOutput);
  }

  public double convertEncoderVelocityToFlywheelVelocity(int ticks) {
    return (ticks * 10. * 60. * (1. / Constants.SHOOTER_MOTOR_TICKS_PER_ROTATION) * Constants.SHOOTER_MOTOR_TO_FLYWHEEL_RATIO);
  }

  public int convertFlywheelVelocityToEncoderVelocity(double rpm) {
    return (int) (rpm * (1. / 10.) * (1. / 60.) * Constants.SHOOTER_MOTOR_TICKS_PER_ROTATION * (1. / Constants.SHOOTER_MOTOR_TO_FLYWHEEL_RATIO));
  }

  @Override
  public void periodic() {
    m_loopCounter += 1;
    SmartDashboard.putNumber("Flywheel velocity", convertEncoderVelocityToFlywheelVelocity(m_flywheelMaster.getSelectedSensorVelocity()));
    if(m_loopCounter % 3. == 0) {
      System.out.println(lastSpeed);
      System.out.println(convertEncoderVelocityToFlywheelVelocity((int)m_flywheelMaster.getClosedLoopTarget()));
    }
  }
}
