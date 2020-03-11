/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmStow extends CommandBase {
  private Arm arm;
  private BooleanSupplier trenchMode;
  /**
   * Creates a new ArmFullUp.
   */
  public ArmStow(Arm arm, BooleanSupplier trenchMode) {
    this.arm = arm;
    this.trenchMode=trenchMode;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getCurrentArmPosition() < 2) {
      arm.setPercentOutput(trenchMode.getAsBoolean() ? -0.04:-0.01);
    } else {
      arm.setArmPosition(0);
    }
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
