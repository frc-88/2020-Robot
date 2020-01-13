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

import com.revrobotics.ColorSensorV3;

public class ColorSensor extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem
   **/

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 m_colorSensor;
  
  public ColorSensor() {
    m_colorSensor = new ColorSensorV3(i2cPort);

  }

  public void updateDashboard(){
    Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("Color: Red Value: ", detectedColor.red);
    SmartDashboard.putNumber("Color: Blue Value: ", detectedColor.blue);
    SmartDashboard.putNumber("Color: Green Value: ", detectedColor.green);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
