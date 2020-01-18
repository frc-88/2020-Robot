/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.ColorShim;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/*
Capturing colors
Spinning the wheel like lightening
Scoring juicy points
*/

public class ColorSensor extends SubsystemBase {
  /**
   * Creates the subsystem for manipulating the color wheel
   **/

  /* TODO: Calibrate our color sensor to find the best target values for each color
    These are just some examples provide by the sensor vendor
  */

  ColorMatch m_colorMatcher = new ColorMatch();
  
  // Get started with some sample colors
  Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  public ColorSensor () {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget); 
  }

  public void testColorSensor(){
    final Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("Color: Red Value: ", detectedColor.red);
    SmartDashboard.putNumber("Color: Blue Value: ", detectedColor.blue);
    SmartDashboard.putNumber("Color: Green Value: ", detectedColor.green);
  }

  private Color getCurrentColorFromSensor(){
    return m_colorSensor.getColor();
  }

  public Color getColorFromGameSession() {
    // This is just a stub that can return a color to see if we've matched something.
    // Will turn this into what we need to get the value from FRC and turn that letter into a color
    // For now we'll try to match yellow

    Color m_colorToMatch = new ColorShim(0.361f, 0.524f, 0.113f);
    return m_colorToMatch;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
