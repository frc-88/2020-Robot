/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cpm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelManipulator;

public class CPMTestGoToPosition extends CommandBase {
  private ControlPanelManipulator cpm;
  private int state;
  private double m_colorWheelRotations;

  /**
   * Creates a new SetWheelPosition.
   */
  public CPMTestGoToPosition(ControlPanelManipulator cpm, double colorWheelRotations) {
    m_colorWheelRotations = colorWheelRotations;
    this.cpm = cpm;
    addRequirements(cpm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case 0: //deploy the CPM
        cpm.deployCPM();
        state = 1;
        break;
      case 1: 
        if(cpm.isCPMDeployed()) {
          //controller.startLightRumble();
          state = 2;
        } else {
          cpm.deployCPM();
        }
        break;
      case 2: //freezes drive, move to case 2 when stopped
        //controller.stopRumble();
        // TODO: take control from the driver
        if(cpm.getMotorSensorPosition() !=0){
          cpm.setMotorSensorPosition(0);
        } else {
          state = 3;
        }
        break;
      case 3: //start spinning motor, spins motor extra distance to target color if already received
        cpm.moveWheelToPosition(m_colorWheelRotations);
        state = 4;
        break;
      case 4: //give control back to the driver + rumble 
        //controller.startHeavyRumble();
        // TODO: give back control to driver
        state = 5;
        break;
      case 5: //stop heavy rumble after driver gets control back
        cpm.retractCPM();
        //controller.stopRumble();
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == 5);
  }
}
