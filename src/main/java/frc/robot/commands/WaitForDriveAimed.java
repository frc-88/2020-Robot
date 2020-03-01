/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class WaitForDriveAimed extends CommandBase {

  private final Drive drive;

  /**
   * Creates a new WaitForShooterReady.
   */
  public WaitForDriveAimed(Drive drive) {
    this.drive = drive;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Drive On Target", this.drive.isOnLimelightTarget());
    return this.drive.isOnLimelightTarget();
  }
}
