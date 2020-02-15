/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private TalonSRX m_feederLeft, m_feederRight;
  private TalonSRXConfiguration m_config;

  /**
   * we can count to five
   * which differs from a certain
   * holy hand grenade
   */
  public Hopper() {
    m_config = new TalonSRXConfiguration();
    m_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
 
    m_feederLeft = new TalonSRX(Constants.HOPPER_FEEDER_1);
    m_feederLeft.configFactoryDefault();
    m_feederLeft.configAllSettings(m_config);

    m_feederRight = new TalonSRX(Constants.HOPPER_FEEDER_2);
    m_feederRight.configFactoryDefault();
    m_feederRight.configAllSettings(m_config);
  }

  public void setFeeders(double leftPercentOutput, double rightPercentOutput) {
    m_feederLeft.set(ControlMode.PercentOutput, leftPercentOutput);
    m_feederRight.set(ControlMode.PercentOutput, rightPercentOutput);
  }

  @Override
  public void periodic() {
    // left motor data
    SmartDashboard.putNumber("Hopper left position", m_feederLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hopper left velocity", m_feederLeft.getSelectedSensorVelocity());

    // right motor data
    SmartDashboard.putNumber("Hopper right position", m_feederRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hopper right velocity", m_feederRight.getSelectedSensorVelocity());
  }
}
