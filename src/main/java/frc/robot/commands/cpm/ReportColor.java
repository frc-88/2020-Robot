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

public class ReportColor extends CommandBase {
  private ControlPanelManipulator cpm;
  /**
   * Creates a new ReportColor.
   */
  public ReportColor(ControlPanelManipulator cpm) {
    this.cpm=cpm;
    addRequirements(cpm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("CPM sensor color", cpm.getColor());
    SmartDashboard.putNumber("Red", cpm.getRawColor().red);
    SmartDashboard.putNumber("Green", cpm.getRawColor().green);
    SmartDashboard.putNumber("Blue", cpm.getRawColor().blue);
    SmartDashboard.putBoolean("CPM Contact Sensor", cpm.isCPMEngaged());
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
