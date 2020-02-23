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

public class TurnToHeading extends CommandBase {

  private static final double TOLERANCE = 0.25;
  private static final int TOLERANCE_TICKS = 5;
  private static final double TOLERANCE_SPEED = 5;

  private Drive drive;
  private Sensors sensors;
  private double heading;

  private int ticksOnTarget = 0;

  /**
   * Creates a new TurnToHeading.
   */
  public TurnToHeading(Drive drive, Sensors sensors, double heading) {
    this.drive = drive;
    this.sensors = sensors;
    this.heading = heading;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ticksOnTarget = 0;
    drive.shiftToLow();
    drive.resetHeadingPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.turnToHeading(heading);
    SmartDashboard.putBoolean("HeadingOnTarget", isOnTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.basicDrive(0, 0);
  }

  public boolean isOnTarget() {
    if (Math.abs(sensors.navx.getYaw() - heading) <= TOLERANCE && sensors.navx.getYawRate() < TOLERANCE_SPEED) {
      ticksOnTarget++;
    } else if (Math.abs(sensors.navx.getYaw() - heading) > TOLERANCE) {
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
