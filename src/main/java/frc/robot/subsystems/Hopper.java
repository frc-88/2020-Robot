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
  private TalonFX m_agitator1, m_agitator2;
  /**
   * Creates a new Hopper.
   */
  public Hopper() {
    m_agitator1 = new TalonFX(Constants.HOPPER_FEEDER_1);
    m_agitator1.configFactoryDefault();

    m_agitator2 = new TalonFX(Constants.HOPPER_FEEDER_2);
    m_agitator2.configFactoryDefault();

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(320, 240);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 320, 240);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        } else {
          // process the image
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
          Core.inRange(output, new Scalar(20, 150, 150),
            new Scalar(50, 255, 255), output);
          for (int i=0; i < Constants.HOPPER_IMAGE_WIDTH; i++) {
            for (int y=0; y < Constants.HOPPER_IMAGE_HEIGHT; i++) {
              output.get(i, y);
            }
          }
        }
        outputStream.putFrame(output);
      }
    }).start();
  
  }

  
	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
  Mat out) {
}

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
