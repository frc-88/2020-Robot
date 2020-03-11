/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonSRX m_feeder = new TalonSRX(Constants.FEEDER_MOTOR);
  private DoubleSolenoid m_deployer = new DoubleSolenoid(Constants.FEEDER_CPM_PCM, Constants.FEEDER_CPM_PISTON_FORWARD, Constants.FEEDER_CPM_PISTON_REVERSE);

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    m_feeder.configFactoryDefault();
    m_feeder.enableVoltageCompensation(true);
    m_feeder.setInverted(false);
    m_feeder.setSensorPhase(false);
    m_feeder.setNeutralMode(NeutralMode.Brake);

    SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration();
    currentLimit.enable = true;
    currentLimit.triggerThresholdTime = 0.001;
    currentLimit.triggerThresholdCurrent = 55;
    currentLimit.currentLimit = 55;
    m_feeder.configSupplyCurrentLimit(currentLimit);
  }

  public void setFeeder(double percentOutput) {
    m_feeder.set(ControlMode.PercentOutput, percentOutput);
  }

  public void retractCPM() {
    m_deployer.set(Value.kReverse);
  }

  public void deployCPM() {
    m_deployer.set(Value.kForward);
  }

  @Override
  public void periodic() {
    retractCPM();
    // This method will be called once per scheduler run
  }
}
