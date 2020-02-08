/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.util.TJController;

public class RotateColorWheel extends CommandBase {
  private ControlPanelManipulator cpm;
  private TJController controller;
  private int state;
  private String targetColor;
  private double finalRotationDistance;
  private double directionCorrection = 3./8.; //if calcPositionControlTargetPosition returns a negative value
  
  /**
   * Creates a new ReportColor.
   */

  public RotateColorWheel(ControlPanelManipulator cpm, TJController controller) {
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
    // If we are in phase 2 and we are in contact with the color wheel 
    // then rotate color wheel 4 times and rumble the controller when done
    // If not phase 2 then just exit and rumble the controller
    
    
    switch(state) {
      case 0: //wait to be engaged
        if(cpm.isEngaged()) {
          controller.startLightRumble();
          state = 1;
        }
        break;
      case 1: //freezes drive, move to case 2 when stopped
        controller.stopRumble();
        // TODO: take control from the driver
        cpm.setWheelPosition(0);
        cpm.getColor();
        state = 2;
        break;
      case 2: //start spinning motor, spins motor extra distance to target color if already received
        targetColor = cpm.getFMSColorTarget();
        finalRotationDistance = Constants.CPM_PHASE_2_WHEEL_ROTATIONS;
        if(targetColor.length() == 1) { //correction distance
          if(cpm.calcPositionControlTargetPosition() < 0) {
            finalRotationDistance =  directionCorrection + Constants.CPM_PHASE_2_WHEEL_ROTATIONS;
          } else {
            finalRotationDistance =  cpm.calcPositionControlTargetPosition() + Constants.CPM_PHASE_2_WHEEL_ROTATIONS;
          }
        }
        cpm.moveWheelToPosition(finalRotationDistance);
        state = 3;
        break;
      case 3: //give control back to the driver + rumble 
        controller.startHeavyRumble();
        // TODO: give back control to driver
        state = 4;
        break;
      case 4: //stop heavy rumble after driver gets control back
        controller.stopRumble();
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
