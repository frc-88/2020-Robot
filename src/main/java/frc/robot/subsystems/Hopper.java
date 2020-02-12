/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private TalonSRX m_feederLeft, m_feederRight;

  /**
   * Creates a new Hopper.
   */
  public Hopper() {
    m_feederLeft = new TalonSRX(Constants.HOPPER_FEEDER_1);
    m_feederLeft.configFactoryDefault();

    m_feederRight = new TalonSRX(Constants.HOPPER_FEEDER_2);
    m_feederRight.configFactoryDefault();
  }

  public void setFeeders(double leftPercentOutput, double rightPercentOutput) {
    m_feederLeft.set(ControlMode.PercentOutput, leftPercentOutput);
    m_feederRight.set(ControlMode.PercentOutput, rightPercentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
