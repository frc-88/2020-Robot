/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {
  private Climber climber;
  private DoubleSupplier xSpeed;
  private DoubleSupplier ySpeed;
  /**
   * Creates a new RunClimber.
   */
  public RunClimber(Climber climber, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    this.climber=climber;
    this.xSpeed=xSpeed;
    this.ySpeed=ySpeed;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tilt;
    double speed;

    //Sets speed value
    speed = ySpeed.getAsDouble();

    //Sets tilt value
    tilt = xSpeed.getAsDouble();

    climber.setRightMotor(speed + tilt);
    climber.setLeftMotor(speed - tilt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
