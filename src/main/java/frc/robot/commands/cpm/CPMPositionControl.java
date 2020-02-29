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

public class CPMPositionControl extends CommandBase {
  private ControlPanelManipulator cpm;
  private int state;
  

  public CPMPositionControl(ControlPanelManipulator cpm) {
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
    // If we are in phase 3 and we are in contact with the color wheel 
    // then rotate color wheel to the color that we are given by the FMS
    switch(state) {
      case 0: // wait to be engaged and have a color from the FMS
        System.out.println("\nCPM: In state "+ state);
        if((cpm.getFMSColorTarget().length()>0)) {
          //controller.startLightRumble();
          state = 1;
          System.out.println("\nCPM: Has FRC Color - New State" + state);
        }
        break;
      case 1: // freezes drive, move to case 2 when stopped
        //controller.stopRumble();
        // TODO: take control from the driver
        System.out.println("\nCPM: In state "+ state);
        // Make sure that the sensor postion returns 0 before moving on
        if(cpm.getMotorSensorPosition() !=0){
          cpm.setMotorSensorPosition(0);
          break;
        }
        state = 2;
        break;
      case 2: // start spinning motor, spins motor extra distance to target color if already received
        System.out.println("\nCPM: In state "+ state);
        System.out.println("\nCPM: Target position: "+ cpm.calcPositionControlTargetPosition());
        cpm.moveWheelToPosition(cpm.calcPositionControlTargetPosition());
        if (cpm.getMotorVelocity() == 0){ // The motor has stopped moving,and we motor is in break mode
          state = 3;
          break;
        }
        break;
      case 3: // give control back to the driver + rumble 
        //controller.startHeavyRumble();
        System.out.println("\nCPM: MoveColorWheelToTargetColor in state: "+ state);
        // TODO: give back control to driver
        state = 4;
        break;
      case 4: // stop heavy rumble after driver gets control back
        //controller.stopRumble();
        System.out.println("\nCPM: MoveColorWheelToTargetColor in state: "+ state);
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
