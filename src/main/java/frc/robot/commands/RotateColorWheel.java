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

public class RotateColorWheel extends CommandBase {
  private ControlPanelManipulator cpm;
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
    // TODO: deploy the CPM
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If we are in phase 2 and we are in contact with the color wheel 
    // then rotate color wheel 4 times and rumble the controller when done
    // If not phase 2 then just exit and rumble the controller
    
    cpm.spinColorWheelNumRotations(Constants.CPM_PHASE_2_WHEEL_ROTATIONS);
    // TODO: rumble the controller
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
