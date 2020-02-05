/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private TalonFX m_feeder1, m_feeder2;
  private double yellow;
  /**
   * Creates a new Hopper.
   */
  public Hopper() {
    m_feeder1 = new TalonFX(Constants.HOPPER_FEEDER_1);
    m_feeder1.configFactoryDefault();

    m_feeder2 = new TalonFX(Constants.HOPPER_FEEDER_2);
    m_feeder2.configFactoryDefault();

    new Thread(() -> {
      int count;
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(Constants.HOPPER_IMAGE_WIDTH, Constants.HOPPER_IMAGE_HEIGHT);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", Constants.HOPPER_IMAGE_WIDTH, Constants.HOPPER_IMAGE_HEIGHT);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        } else {
          // process the image
          count = 0;
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
          Core.inRange(output, new Scalar(20, 150, 150),
            new Scalar(50, 255, 255), output);
          for (int i=0; i < Constants.HOPPER_IMAGE_WIDTH; i++) {
            for (int y=0; y < Constants.HOPPER_IMAGE_HEIGHT; i++) {
              double [] pixel = output.get(i, y);
              if (pixel[0] == 255) {
                count++;
              }
            }
          }
          yellow = count / (Constants.HOPPER_IMAGE_HEIGHT * Constants.HOPPER_IMAGE_WIDTH);
        }
      }
    }).start();
  
  }

  public double getYellowPercentage() {
    return yellow;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
