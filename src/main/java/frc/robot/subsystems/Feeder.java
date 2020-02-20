/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonSRX m_feeder = new TalonSRX(Constants.FEEDER_MOTOR);

  /**
   * Creates a new Feeder.
   */
  public Feeder() {
    m_feeder.configFactoryDefault();
    m_feeder.enableVoltageCompensation(true);
    m_feeder.setInverted(false);
    m_feeder.setSensorPhase(false);
    m_feeder.setNeutralMode(NeutralMode.Brake);
  }

  public void setFeeder(double percentOutput) {
    m_feeder.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
