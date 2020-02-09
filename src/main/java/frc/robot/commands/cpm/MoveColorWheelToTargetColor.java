/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
/* Takes the color give to us by the FMS and then moves the large color wheel
/* to that new position
*/
package frc.robot.commands.cpm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.util.TJController;

public class MoveColorWheelToTargetColor extends CommandBase {
  private ControlPanelManipulator cpm;
  private TJController controller;
  private int state;
  

  public MoveColorWheelToTargetColor(ControlPanelManipulator cpm, TJController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cpm=cpm;
    this.controller=controller;
    addRequirements(cpm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    // TODO: deploy the CPM
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If we are in phase 3 and we are in contact with the color wheel 
    // then rotate color wheel to the color that we are given by the FMS
    
    switch(state) {
      case 0: // wait to be engaged and have a color from the FMS
        if((cpm.isEngaged())&&(cpm.getFMSColorTarget().length()>0)) {
          controller.startLightRumble();
          state = 1;
          System.out.println("CPM: MoveColorWheelToTargetColor "+ Integer.toString(state));
        }
        System.out.println("CPM: MoveColorWheelToTargetColor "+Integer.toString(state));
        break;
      case 1: // freezes drive, move to case 2 when stopped
        controller.stopRumble();
        // TODO: take control from the driver
        cpm.setWheelPosition(0);
        cpm.getColor();
        System.out.println("CPM: MoveColorWheelToTargetColor "+Integer.toString(state));
        state = 2;
        break;
      case 2: // start spinning motor, spins motor extra distance to target color if already received
        cpm.moveWheelToPosition(cpm.calcPositionControlTargetPosition());
        System.out.println("CPM: MoveColorWheelToTargetColor "+Integer.toString(state));
        state = 3;
        break;
      case 3: // give control back to the driver + rumble 
        controller.startHeavyRumble();
        System.out.println("CPM: MoveColorWheelToTargetColor "+Integer.toString(state));
        // TODO: give back control to driver
        state = 4;
        break;
      case 4: // stop heavy rumble after driver gets control back
        controller.stopRumble();
        System.out.println("CPM: MoveColorWheelToTargetColor in state: "+Integer.toString(state));
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
