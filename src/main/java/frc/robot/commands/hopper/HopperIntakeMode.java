/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

/**
 * Run each side slowly in opposite directions until a ball 
 * is found in the center (nearest shooter) position
 */
public class HopperIntakeMode extends CommandBase {

  private static final long TIME_BETWEEN_CHANGES = 1 * 1_000_000;

  private Hopper m_hopper;
  private double m_percentOutput;

  private int currentDirection;
  private long lastChangeTime;

  public HopperIntakeMode(Hopper hopper) {
    this(hopper, Constants.HOPPER_INTAKE_PERCENT_OUTPUT);
  }

  public HopperIntakeMode(Hopper hopper, double percentOutput) {
    m_hopper = hopper;
    m_percentOutput = percentOutput;
    addRequirements(m_hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDirection = 1;
    lastChangeTime = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO - Add code like this if we can detect is power cell is ready
    // if (m_hopper.isPowerCellReady()) {
    //   m_hopper.setFeeders(0, 0);
    // } else {
    // m_hopper.setFeeders(-m_percentOutput, m_percentOutput);
    // }

    if (RobotController.getFPGATime() - lastChangeTime >= TIME_BETWEEN_CHANGES) {
      currentDirection *= -1;
      lastChangeTime = RobotController.getFPGATime();
    }

    m_hopper.setFeeders(currentDirection * m_percentOutput, -currentDirection * m_percentOutput);
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
