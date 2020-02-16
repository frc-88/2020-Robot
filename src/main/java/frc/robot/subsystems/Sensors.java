/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;
import frc.robot.util.NavX;

/**
 * we gather data
 * about the world around us
 * to make good choices
 */

public class Sensors extends SubsystemBase {
  public final NavX m_navx;
  public final Limelight m_limelight;

  private CameraServer cameraServer = CameraServer.getInstance();
  private UsbCamera frontCamera, rearCamera, hopperCamera;

  private double m_totalYellow = 0.0;
  private double m_totalYellowChamber = 0.0;
  private boolean m_cellInChamber = false;

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
    m_navx = new NavX();
    m_navx.zeroYaw();

    m_limelight = new Limelight();
    m_limelight.camVision();
    m_limelight.ledOn();

    frontCamera = cameraServer.startAutomaticCapture(0);
    rearCamera = cameraServer.startAutomaticCapture(1);
    hopperCamera = cameraServer.startAutomaticCapture(Constants.PCC_CAMERA_NAME, Constants.PCC_CAMERA_ID);
    
    setToFrontCamera();
    // frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // rearCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    startCounter(hopperCamera);
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
        Imgproc.blur(output, output, new Size(32, 32));
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

  public void setToFrontCamera() {
    cameraServer.getServer().setSource(frontCamera);
  }

  public void setToRearCamera() {
    cameraServer.getServer().setSource(rearCamera);
  }

  public boolean isCellInChamber() {
    return m_cellInChamber;
  }

  @Override
  public void periodic() {
    // NavX data
    SmartDashboard.putNumber("NavX Yaw", m_navx.getYaw());
    SmartDashboard.putNumber("NavX Yaw Rate", m_navx.getYawRate());
    SmartDashboard.putNumber("NavX Pitch", m_navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", m_navx.getRoll());

    // Limelight data
    SmartDashboard.putBoolean("Limelight connected?", m_limelight.isConnected());
    SmartDashboard.putBoolean("Limelight has target?", m_limelight.hasTarget());

    // PCC data
    SmartDashboard.putNumber("PCC Total Yellow", m_totalYellow);
    SmartDashboard.putBoolean("PCC Chamber Loaded?", m_cellInChamber);
    SmartDashboard.putNumber("PCC Total Yellow (Chamber)", m_totalYellowChamber);
  }
}
