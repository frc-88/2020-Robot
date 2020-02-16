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

  UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
  UsbCamera rearCamera = CameraServer.getInstance().startAutomaticCapture(1);
  VideoSink server = CameraServer.getInstance().getServer();
  private double m_totalYellow = 0.0;

  /**
   * Creates a new Sensors subsystem
   */
  public Sensors() {
    m_navx = new NavX();
    m_navx.zeroYaw();

    m_limelight = new Limelight();
    m_limelight.camVision();
    m_limelight.ledOn();

    setToFrontCamera();
    // frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // rearCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    startCounter(frontCamera);
  }

  public void startCounter(UsbCamera camera) {
    new Thread(() -> {
      camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();
      Mat hierarchy = new Mat();
      List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
        double area = 0.0;
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
				Imgproc.blur(output, output, new Size(32, 32));
        Core.inRange(output, new Scalar(10, 0, 0),
              new Scalar(60, 255, 255), output);
        contours.clear();        
        Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour: contours) {
          area += Imgproc.contourArea(contour);
        }

        m_totalYellow = area;
        outputStream.putFrame(output);
      }
    }).start();
  }


  public void setToFrontCamera() {
    server.setSource(frontCamera);
  }

  public void setToRearCamera() {
    server.setSource(rearCamera);
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
  }
}
