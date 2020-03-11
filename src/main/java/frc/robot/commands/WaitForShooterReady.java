/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class WaitForShooterReady extends CommandBase {

  private final Arm arm;
  private final Shooter shooter;

  private int debounceTicks;

  private final int DEBOUNCE = 5;
  
  /**
   * Creates a new WaitForShooterReady.
   */
  public WaitForShooterReady(Arm arm, Shooter shooter) {
    this.arm = arm;
    this.shooter = shooter;
  }

  public void initialize() {
    debounceTicks = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Arm On Target", this.arm.isOnTarget());
    SmartDashboard.putBoolean("Shooter On Target", this.shooter.flywheelOnTarget());
    if (this.arm.isOnTarget() && this.shooter.flywheelOnTarget()) {
      debounceTicks++;
    } else {
      debounceTicks = 0;
    }
    return debounceTicks >= DEBOUNCE;
  }
}
