/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {
  private final AHRS m_navx;
  private double m_pitchOffset;
  private double m_rollOffset;

    /**
   * Creates a new Sensors.
   */
  public Sensors() {
    m_navx = new AHRS(Port.kMXP);

    zeroPitch();
    zeroRoll();
    zeroYaw();
  }

  public void zeroYaw() {
    m_navx.zeroYaw();
  }

  public void zeroPitch() {
    m_pitchOffset = m_navx.getPitch();
  }
  public void zeroRoll() {
    m_rollOffset = m_navx.getRoll();
  }

  public double getYaw() {
    return m_navx.getYaw();
  }

  public double getPitch() {
    return m_navx.getPitch() - m_pitchOffset;
  }

  public double getRoll() {
    return m_navx.getRoll() - m_rollOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // NavX data
    SmartDashboard.putNumber("NavX Yaw", getYaw());
    SmartDashboard.putNumber("NavX Pitch", getPitch());
    SmartDashboard.putNumber("NavX Roll", getRoll());
  }
}
