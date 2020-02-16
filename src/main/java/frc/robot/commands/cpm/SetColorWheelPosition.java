/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cpm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.util.TJController;

public class SetColorWheelPosition extends CommandBase {
  private ControlPanelManipulator cpm;
  private TJController controller;
  private int state;

  /**
   * Creates a new SetWheelPosition.
   */
  public SetColorWheelPosition(ControlPanelManipulator cpm, TJController controller) {
    SmartDashboard.putNumber("CPM Desired Wheel Position", 0);
    this.cpm = cpm;
    addRequirements(cpm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case 0: //wait to be engaged
        if(cpm.isEngaged()) {
          //controller.startLightRumble();
          state = 1;
        }
        break;
      case 1: //freezes drive, move to case 2 when stopped
        //controller.stopRumble();
        // TODO: take control from the driver
        cpm.setSensorPosition(0);
        state = 2;
        break;
      case 2: //start spinning motor, spins motor extra distance to target color if already received
        cpm.moveWheelToPosition(SmartDashboard.getNumber("CPM Desired Wheel Position", 0));
        state = 3;
        break;
      case 3: //give control back to the driver + rumble 
        //controller.startHeavyRumble();
        // TODO: give back control to driver
        state = 4;
        break;
      case 4: //stop heavy rumble after driver gets control back
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
    return (state == 4);
  }
}
