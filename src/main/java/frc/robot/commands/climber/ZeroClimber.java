/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends CommandBase {
  private Climber climber;
  private long startTime;
  /**
   * Creates a new ZeroClimber.
   */
  public ZeroClimber(Climber climber) {
    this.climber=climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.enterZeroMode();
    startTime = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setMotors(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.exitZeroMode();
    climber.setMotors(0);
    climber.zero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return RobotController.getFPGATime() - startTime >= 10 * 1_000_000;
  }
}
