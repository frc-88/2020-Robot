/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.NavX;
import frc.robot.Constants;
import frc.robot.util.Limelight;

/**
 * we gather data
 * about the world around us
 * to make good choices
 */

public class Sensors extends SubsystemBase {
  public final NavX navx;
  
  private final Limelight limelight;
  private BooleanSupplier ledOverride;

  UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
  UsbCamera rearCamera = CameraServer.getInstance().startAutomaticCapture(1);
  VideoSink server = CameraServer.getInstance().getServer();

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors(BooleanSupplier ledOverride) {
    this.ledOverride = ledOverride;

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

  public void ledOn() {
    limelight.ledOn();
  }

  public void ledOff() {
    if(!(DriverStation.getInstance().isDisabled() && ledOverride.getAsBoolean())) {
      limelight.ledOff();
    }
  }

  public double getDistanceToTarget() {
    double distance = 0;

    if (limelight.isConnected() && limelight.hasTarget()) {
      distance = (Constants.FIELD_PORT_TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT) / 
        Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE) + Math.toRadians(limelight.getTargetVerticalOffsetAngle()));
    }

    return distance;
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
    SmartDashboard.putNumber("Distance to target", getDistanceToTarget());
    SmartDashboard.putNumber("Angle to target", limelight.getTargetHorizontalOffsetAngle());

    // Check LED override, only when disabled
    if(DriverStation.getInstance().isDisabled() && ledOverride.getAsBoolean()) {
      limelight.ledOn();
    }
  }
}
