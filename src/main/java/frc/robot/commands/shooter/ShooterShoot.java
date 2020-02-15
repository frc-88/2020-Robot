/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterShoot extends ShooterFlywheelRun {
  private Shooter m_shooter;
  private double m_feederPercentOutput;
  
  /**
   * Sets feeder to desired percent output
   */
  public ShooterShoot(Shooter shooter, double shooterRPM, double feederPercentOutput) {
    super(shooter, shooterRPM);
    this.m_feederPercentOutput = feederPercentOutput;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setFeeder(m_feederPercentOutput);
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setFeeder(0);
    super.execute();
  }
}
