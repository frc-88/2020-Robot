/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
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
 
    m_feederLeft = new TalonSRX(Constants.LEFT_HOPPER);
    m_feederLeft.configFactoryDefault();
    m_feederLeft.configAllSettings(m_config);
    m_feederLeft.setInverted(InvertType.None);

    m_feederRight = new TalonSRX(Constants.RIGHT_HOPPER);
    m_feederRight.configFactoryDefault();
    m_feederRight.configAllSettings(m_config);
    m_feederRight.setInverted(InvertType.None);
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
    SmartDashboard.putNumber("Hopper left stator current", m_feederLeft.getStatorCurrent());
    SmartDashboard.putNumber("Hopper left supply current", m_feederLeft.getSupplyCurrent());

    // right motor data
    SmartDashboard.putNumber("Hopper right position", m_feederRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hopper right velocity", m_feederRight.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Hopper right stator current", m_feederRight.getStatorCurrent());
    SmartDashboard.putNumber("Hopper right supply current", m_feederRight.getSupplyCurrent());
  }
}
