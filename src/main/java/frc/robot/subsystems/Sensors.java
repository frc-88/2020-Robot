/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.NavX;
import frc.robot.util.Limelight;

public class Sensors extends SubsystemBase {
  public final NavX m_navx;
  public final Limelight m_limelight;

  UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
  UsbCamera rearCamera = CameraServer.getInstance().startAutomaticCapture(1);
  VideoSink server = CameraServer.getInstance().getServer();

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
    m_navx = new NavX();
    m_navx.zeroYaw();

    m_limelight = new Limelight();
    m_limelight.camVision();
    m_limelight.ledOff();

    setToFrontCamera();
    // frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // rearCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  public void setToFrontCamera() {
    server.setSource(frontCamera);
  }

  public void setToRearCamera() {
    server.setSource(rearCamera);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // NavX data
    SmartDashboard.putNumber("NavX Yaw", m_navx.getYaw());
    SmartDashboard.putNumber("NavX Yaw Rate", m_navx.getYawRate());
    SmartDashboard.putNumber("NavX Pitch", m_navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", m_navx.getRoll());

    // Limelight data
    SmartDashboard.putBoolean("Limelight connected?", m_limelight.isConnected());
    SmartDashboard.putBoolean("Limelight has target?", m_limelight.hasTarget());
  }
}
