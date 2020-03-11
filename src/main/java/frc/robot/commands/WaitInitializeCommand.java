/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitInitializeCommand extends CommandBase {

  private final DoubleSupplier waitSupplier;
  private long waitTime;
  private long startTime;

  /**
   * Creates a new WaitIntializeCommand.
   */
  public WaitInitializeCommand(final DoubleSupplier seconds) {
    waitSupplier = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    waitTime = (long)(waitSupplier.getAsDouble() * 1_000_000);
    startTime = RobotController.getFPGATime();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotController.getFPGATime() - startTime >= waitTime;
  }
}
