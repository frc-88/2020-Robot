/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import.java.util.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ControlPanelManipulator extends SubsystemBase {
  /**
   * Creates a new ControlPanelManipulator.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final float controlPanelSlice = 12.5f;

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private TalonFX m_spinner = new TalonFX(Constants.CPMMotor);
  private final I2C.Port m_i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(m_i2cPort);
  private Encoder m_wristEncoder = new Encoder(Constants.CPMJointEncoderChannelA, Constants.CPMJointEncoderChannelB);
  private DoubleSolenoid m_pneumatics = new DoubleSolenoid(Constants.CPMPneumaticsForward, Constants.CPMPneumaticsReverse);
  private AHRS m_navX = new AHRS(SPI.Port.kMXP); 
  private DigitalInput m_contactSensor = new DigitalInput(Constants.CPMDigitalInputChannel);
  

  public ControlPanelManipulator() {
    m_wristEncoder.reset();
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    
  }

  public boolean isEngaged() {
    return m_contactSensor.get();
  }

  public String getColor() {
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "b";
    } else if (match.color == kRedTarget) {
      colorString = "r";
    } else if (match.color == kGreenTarget) {
      colorString = "g";
    } else if (match.color == kYellowTarget) {
      colorString = "y";
    } else {
      colorString = "?";
    }
    return colorString;
  }

  public String getFMSColorTarget() {
    return DriverStation.getInstance().getGameSpecificMessage();
  }

  public double getRobotFacing() {
    return m_navX.getYaw();
  }

  public double getRobotWrist() {
    return m_wristEncoder.get()/Constants.CPMWristEncoderCountsPerRev*360;
  }

  public float calcPositionControlTargetPosition() {
    //calculate in inches, motor can spin x inches
    int colorDistance = 0;
    int i = 1; //Start at index 1
    String currentDetectedColor = getColor();
    String currentTargetColor = getFMSColorTarget();
    String[] colorList = { "r", "g", "b", "y", "r", "g"};
    int colorListLength = colorList.length;
    while (i < colorListLength) {
      if (colorList[i] == currentDetectedColor) {
        if (colorList[i-1] == currentTargetColor) {
          colorDistance = -1;
          break;
        }
        else if (colorList[i+1] == currentTargetColor) {
          colorDistance = 1;
          break;
        }
        else {
          colorDistance = 2;
          break;
        }
      }
    }
    return colorDistance*12.5f;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
