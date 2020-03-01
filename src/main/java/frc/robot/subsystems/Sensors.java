/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private CameraServer cameraServer = CameraServer.getInstance();
  // private UsbCamera intakeCamera, hopperCamera;

  private double m_totalYellow = 0.0;
  private double m_totalYellowChamber = 0.0;
  private boolean m_cellInChamber = false;

  private DigitalInput shooterBallSensor;

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

    shooterBallSensor = new DigitalInput(Constants.SHOOTER_BALL_SENSOR_ID);
    // intakeCamera = cameraServer.startAutomaticCapture(0);
    // hopperCamera = cameraServer.startAutomaticCapture(Constants.PCC_CAMERA_NAME, Constants.PCC_CAMERA_ID);
    
    // startCounter(hopperCamera);

    // cameraServer.getServer().setSource(intakeCamera);
  }

  public void startCounter(UsbCamera camera) {
    new Thread(() -> {
      camera.setResolution(Constants.PCC_IMAGE_WIDTH, Constants.PCC_IMAGE_HEIGHT);

      CvSink cvSink = cameraServer.getVideo(Constants.PCC_CAMERA_NAME);
      CvSource outputStream = cameraServer.putVideo(Constants.PCC_STREAM_NAME, 
      Constants.PCC_IMAGE_WIDTH,
          Constants.PCC_IMAGE_HEIGHT);

      Mat source = new Mat();
      Mat output = new Mat();
      Mat hierarchy = new Mat();
      Mat chamber;

      List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
        double area = 0.0;
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
        Imgproc.blur(output, output, new Size(Constants.PCC_BLUR, Constants.PCC_BLUR));
        Core.inRange(output, new Scalar(Constants.PCC_HUE_LO, Constants.PCC_SAT_LO, Constants.PCC_VAL_LO),
            new Scalar(Constants.PCC_HUE_HI, Constants.PCC_SAT_HI, Constants.PCC_VAL_HI), output);
        contours.clear();
        Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
          area += Imgproc.contourArea(contour);
        }
        m_totalYellow = area;

        area = 0.0;
        chamber = new Mat(output, new Rect(Constants.PCC_CHAMBER_X, Constants.PCC_CHAMBER_Y,
            Constants.PCC_CHAMBER_WIDTH, Constants.PCC_CHAMBER_HEIGHT));
        contours.clear();
        Imgproc.findContours(chamber, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
          area += Imgproc.contourArea(contour);
        }

        m_totalYellowChamber = area;
        m_cellInChamber = area > Constants.PCC_CHAMBER_THRESHOLD;

        outputStream.putFrame(output);
      }
    }).start();
  }

  public void ledOn() {
    limelight.ledOn();
  }

  public void ledOff() {
    if(!(DriverStation.getInstance().isDisabled() && ledOverride.getAsBoolean())) {
      limelight.ledOff();
    }
  }

  public boolean isCellInChamber() {
    return m_cellInChamber;
  }
  
  public double getDistanceToTarget() {
    double distance = 0;

    if (limelight.isConnected() && limelight.hasTarget()) {
      // distance = (Constants.FIELD_PORT_TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT) / 
      //   Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + limelight.getTargetVerticalOffsetAngle()));
      //
      // After further analysis of the spreadsheet data, 
      // a polynomial curve matches the empirical data best
      // Sorry, Bill, for the magic numbers. They came from
      // a spreadsheet, I swear! :D

      double ty = limelight.getTargetVerticalOffsetAngle();

      distance = 190 - 11.7 * ty + 0.32 * ty * ty;
    }

    return distance;
  }

  public double getAngleToTarget() {
    return -limelight.getTargetHorizontalOffsetAngle();
  }

  public boolean doesLimelightHaveTarget() {
    return limelight.hasTarget();
  }

  @Override
  public void periodic() {
    // NavX data
    SmartDashboard.putNumber("NavX Yaw", navx.getYaw());
    SmartDashboard.putNumber("NavX Yaw Rate", navx.getYawRate());
    SmartDashboard.putNumber("NavX Pitch", navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", navx.getRoll());

    // Limelight data
    SmartDashboard.putBoolean("Limelight connected?", limelight.isConnected());
    SmartDashboard.putBoolean("Limelight has target?", limelight.hasTarget());
    SmartDashboard.putNumber("Limelight Distance", getDistanceToTarget());
    SmartDashboard.putNumber("Limelight H-Angle", getAngleToTarget());
    SmartDashboard.putNumber("Limelight V-Angle", limelight.getTargetVerticalOffsetAngle());

    // Beam breaks
    SmartDashboard.putBoolean("Shooter Ball Sensor", shooterBallSensor.get());

    // Check LED override, only when disabled
    if(DriverStation.getInstance().isDisabled() && ledOverride.getAsBoolean()) {
      limelight.ledOn();
    }

    // PCC data
    SmartDashboard.putNumber("PCC Total Yellow", m_totalYellow);
    SmartDashboard.putBoolean("PCC Chamber Loaded?", m_cellInChamber);
    SmartDashboard.putNumber("PCC Total Yellow (Chamber)", m_totalYellowChamber);
  }
}
