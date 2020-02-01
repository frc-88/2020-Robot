/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private SerialPort serialPort;

  /**
   * Creates a new Lights.
   */
  public Lights() {
    serialPort = new SerialPort(9600, SerialPort.Port.kUSB);
  }

  public void sendOn() {
    serialPort.writeString("on");
  }

  public void sendOff() {
    serialPort.writeString("off");
  }

  public void sendString(String message) {
    serialPort.writeString(message);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // TODO gather status information and send status message to Arduino

    sendString("status");
  }

}
