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
      case 0: // wait to have a color from the FMS and deploy the CPM
        System.out.println("\nCPM: In state "+ state);
        if((cpm.getFMSColorTarget().length()>0)) {
          //controller.startLightRumble();
          cpm.deployCPM();
          System.out.println("\nCPM: Has FRC Color - New State" + state);
        }
        break;
      case 1: // Wait for CPM to be deployed
        if(cpm.isCPMDeployed()) {
          //controller.startLightRumble();
          state = 2;
        } else {
          cpm.deployCPM();
        }
        break;
      case 2: // freezes drive, move to case 2 when stopped
        //controller.stopRumble();
        // TODO: take control from the driver
        System.out.println("\nCPM: In state "+ state);
        // Make sure that the sensor postion returns 0 before moving on
        if(cpm.getMotorSensorPosition() !=0){
          cpm.setMotorSensorPosition(0);
        } else {
          state = 3;
        }
        break;
      case 3: // start spinning motor, spins motor extra distance to target color if already received
        System.out.println("\nCPM: In state "+ state);
        System.out.println("\nCPM: Target position: "+ cpm.calcPositionControlTargetPosition());
        if (Math.abs(cpm.getWheelPosition()) > .95*cpm.calcPositionControlTargetPosition()){
          state = 4;
        } else {
          cpm.moveWheelToPosition(-cpm.calcPositionControlTargetPosition());
        }
        break;
      case 4: // give control back to the driver + rumble 
        //controller.startHeavyRumble();
        System.out.println("\nCPM: MoveColorWheelToTargetColor in state: "+ state);
        // TODO: give back control to driver
        state = 4;
        break;
      case 5: // stop heavy rumble after driver gets control back
        //controller.stopRumble();
        cpm.retractCPM();
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
    return (state == 5);
  }
}
