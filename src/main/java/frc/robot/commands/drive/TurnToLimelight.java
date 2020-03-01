/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Sensors;

public class TurnToLimelight extends CommandBase {

  private static final boolean LOG_DEBUG = true;

  private static final double TOLERANCE = 0.75;
  private static final int TOLERANCE_TICKS = 5;
  private static final double TOLERANCE_SPEED = 5;

  private Drive drive;
  private Sensors sensors;

  private double currentHeadingTarget;
  private int ticksOnTarget = 0;
  private boolean firstRun;

  /**
   * Creates a new TurnToHeading.
   */
  public TurnToLimelight(Drive drive, Sensors sensors) {
    this.drive = drive;
    this.sensors = sensors;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ticksOnTarget = 0;
    drive.setOnLimelightTarget(false);
    firstRun = true;
    drive.shiftToLow();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("LimelightHeadingOnTarget", drive.isOnLimelightTarget());
    if (drive.isOnLimelightTarget()) {
      if (LOG_DEBUG) {
        drive.turnToHeading(currentHeadingTarget);
        System.out.println("On limelight target");
      }
      return;
    }
    if (isOnNavxTarget() || firstRun) {
      if (LOG_DEBUG) {
        System.out.println("On navx target");
      }
      if (!sensors.doesLimelightHaveTarget()) {
        return;
      }
      if (Math.abs(sensors.getShooterAngle()) < TOLERANCE) {
        drive.setOnLimelightTarget(true);
        return;
      }
      firstRun = false;
      currentHeadingTarget = sensors.navx.getYaw() - sensors.getShooterAngle();
      ticksOnTarget = 0;
      drive.resetHeadingPID();

      if (LOG_DEBUG) {
        System.out.println("Limelight horizontal offset: " + sensors.getShooterAngle());
      }
      SmartDashboard.putBoolean("NavxHeadingOnTarget", true);
    } else {
      SmartDashboard.putBoolean("NavxHeadingOnTarget", false);
    }
    drive.turnToHeading(currentHeadingTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setOnLimelightTarget(false);
    drive.basicDrive(0, 0);
  }

  private boolean isOnNavxTarget() {
    System.out.println(sensors.navx.getYaw() - currentHeadingTarget);
    if (Math.abs(sensors.navx.getYaw() - currentHeadingTarget) <= TOLERANCE && sensors.navx.getYawRate() < TOLERANCE_SPEED) {
      ticksOnTarget++;
    } else if (Math.abs(sensors.navx.getYaw() - currentHeadingTarget) > TOLERANCE) {
      ticksOnTarget = 0;
    }
    return ticksOnTarget >= TOLERANCE_TICKS;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
