/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cpm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelManipulator;

public class RotateColorWheel extends CommandBase {
  private ControlPanelManipulator cpm;
  private int state;
  private String targetColor;
  private double finalRotationDistance;
  private double directionCorrection = 3./8.; //if calcPositionControlTargetPosition returns a negative value
  
  /**
   * Creates a new ReportColor.
   */

  public RotateColorWheel(ControlPanelManipulator cpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cpm=cpm;
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
    System.out.println("CPM: Executing the Rotate Color Wheel command");
    
    switch(state) {
      case 0: //wait to be engaged
        System.out.println("CPM RotateColorWheel State: "+ state);
        if(cpm.isCPMEngaged()) {
          //controller.startLightRumble();
          state = 1;
        }
        break;
      case 1: //freezes drive, move to case 2 when stopped
        System.out.println("\nCPM RotateColorWheel State: "+ state);
        //controller.stopRumble();
        // TODO: take control from the driver
        if(cpm.getMotorSensorPosition() !=0){
          cpm.setMotorSensorPosition(0);
          break;
        }
        cpm.getColor();
        state = 2;
        break;
      case 2: //start spinning motor, spins motor extra distance to target color if already received
        System.out.println("\nCPM RotateColorWheel State: "+ state);
        targetColor = cpm.getFMSColorTarget();
        finalRotationDistance = Constants.CPM_PHASE_2_WHEEL_ROTATIONS;
        if(targetColor.length() == 1) { //correction distance
          System.out.println("\nCPM: we got a game color");
          if(cpm.calcPositionControlTargetPosition() < 0) {
            finalRotationDistance =  directionCorrection + Constants.CPM_PHASE_2_WHEEL_ROTATIONS;
            System.out.println("\nCPM: Adding direction correction" + finalRotationDistance);
          } else {
            finalRotationDistance =  cpm.calcPositionControlTargetPosition() + Constants.CPM_PHASE_2_WHEEL_ROTATIONS;
            System.out.println("\nCPM: Distance to the next color" + cpm.calcPositionControlTargetPosition());
          }
        }
        System.out.println("\nCPM target distance: "+ finalRotationDistance);
        cpm.moveWheelToPosition(finalRotationDistance);
        state = 3;
        break;
      case 3: //give control back to the driver + rumble 
        System.out.println("\nCPM RotateColorWheel State: "+ state);
        //controller.startHeavyRumble();
        // TODO: give back control to driver
        state = 4;
        break;
      case 4: //stop heavy rumble after driver gets control back
        System.out.println("\nCPM RotateColorWheel State: "+ state);
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
