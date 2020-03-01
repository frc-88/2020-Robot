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

public class CPMTinyCorrectionRotation extends CommandBase {
  private ControlPanelManipulator cpm;
  private int state;
  /**
   * Creates a new CPMCorrectionRotation
   */

  public CPMTinyCorrectionRotation(ControlPanelManipulator cpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cpm=cpm;
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
    // If we think we're there, but maybe a little short, this will move a tiny bit

    System.out.println("CPM: Executing the tiny correction turn");
    
    switch(state) {
      case 0: 
        if(cpm.isCPMDeployed()) {
          //controller.startLightRumble();
          state = 1;
        } else {
          cpm.deployCPM();
        }
        break;
      case 1: //freezes drive, move to case 2 when stopped
        //controller.stopRumble();
        // TODO: take control from the driver
        if(cpm.getMotorSensorPosition() !=0){
          cpm.setMotorSensorPosition(0);
        } else {
          cpm.getColor();
          state = 2;
        }
        break;
      case 2: //start spinning motor, spins motor extra distance to target color if already received
        cpm.moveWheelToPosition(Constants.CPM_TINY_TURN_ROTATIONS);
        state = 3;
        break;
      case 3: //give control back to the driver + rumble 
        System.out.println("\nCPM RotateColorWheel State: "+ state);
        //controller.startHeavyRumble();
        // TODO: give back control to driver
        if (cpm.getMotorVelocity() == 0){ // motor has stopped moving, motor is in break mode, proceed
          state = 4;
        }
        break;
      case 4: //stop heavy rumble after driver gets control back
        cpm.retractCPM();
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