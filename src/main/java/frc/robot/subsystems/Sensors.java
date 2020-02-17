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

/**
 * we gather data
 * about the world around us
 * to make good choices
 */

public class Sensors extends SubsystemBase {
  public final NavX navx;
  public final Limelight limelight;

  UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
  UsbCamera rearCamera = CameraServer.getInstance().startAutomaticCapture(1);
  VideoSink server = CameraServer.getInstance().getServer();

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
    navx = new NavX();
    navx.zeroYaw();

    limelight = new Limelight();
    limelight.camVision();
    limelight.ledOff();

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
    SmartDashboard.putNumber("NavX Yaw", navx.getYaw());
    SmartDashboard.putNumber("NavX Yaw Rate", navx.getYawRate());
    SmartDashboard.putNumber("NavX Pitch", navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", navx.getRoll());

    // Limelight data
    SmartDashboard.putBoolean("Limelight connected?", limelight.isConnected());
    SmartDashboard.putBoolean("Limelight has target?", limelight.hasTarget());
  }
}
