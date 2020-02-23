/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberSetPosition extends CommandBase {
  private Climber climber;
  private DoubleSupplier xPos;
  private DoubleSupplier yPos;
  /**
   * Creates a new ClimberSetPosition.
   */
  public ClimberSetPosition(Climber climber, DoubleSupplier xPos, DoubleSupplier yPos) {
    this.climber=climber;
    this.xPos=xPos;
    this.yPos=yPos;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tilt;
    double pos;

    tilt = xPos.getAsDouble();
    pos = yPos.getAsDouble();

    climber.setBothMotorHeights(pos - tilt, pos + tilt);
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
