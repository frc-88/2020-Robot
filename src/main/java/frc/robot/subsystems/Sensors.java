/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.NavX;
import frc.robot.util.Limelight;

public class Sensors extends SubsystemBase {
  public final NavX m_navx;
  public final Limelight m_limelight;

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
    m_navx = new NavX();
    m_navx.zeroYaw();

    m_limelight = new Limelight();
    m_limelight.camVision();
    m_limelight.ledOff();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // NavX data
    SmartDashboard.putNumber("NavX Yaw", m_navx.getYaw());
    SmartDashboard.putNumber("NavX Pitch", m_navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", m_navx.getRoll());

    // Limelight data
    SmartDashboard.putBoolean("Limelight connected?", m_limelight.isConnected());
    SmartDashboard.putBoolean("Limelight has target?", m_limelight.hasTarget());
  }
}
