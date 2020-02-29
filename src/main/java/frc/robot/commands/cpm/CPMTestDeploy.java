/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cpm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ControlPanelManipulator;

public class CPMTestDeploy extends InstantCommand {
  private ControlPanelManipulator cpm;
  /**
   * Creates a new CPMTestDeploy.
   */
  public CPMTestDeploy(ControlPanelManipulator cpm) {
    this.cpm=cpm;
    addRequirements(cpm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cpm.deployCPM();
  }
}
