/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ControlPanelManipulator extends SubsystemBase {
  /**
   * Creates a new ControlPanelManipulator.
   */

  private TalonFX m_spinner = new TalonFX(Constants.CPMMotor);
  private final I2C.Port m_i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(m_i2cPort);
  private Encoder m_positionEncoder = new Encoder(Constants.CPMJointEncoderChannelA, Constants.CPMJointEncoderChannelB);
  private DoubleSolenoid m_pneumatics = new DoubleSolenoid(Constants.CPMPneumaticsForward, Constants.CPMPneumaticsReverse);
  private AHRS m_navX = new AHRS(SPI.Port.kMXP); 
  private DigitalInput m_contactSensor = new DigitalInput(Constants.CPMDigitalInputChannel);

  public ControlPanelManipulator() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
