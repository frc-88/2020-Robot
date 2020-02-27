/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class BasicAutoDrive extends CommandBase {

  private final Drive m_drive;
  private final double m_leftDistance;
  private final double m_rightDistance;
  private final double m_maxSpeed;
  /**
   * Creates a new BasicAutoDrive.
   */
  public BasicAutoDrive(final Drive drive, final double leftDistance, final double rightDistance, final double maxSpeed) {
    m_drive = drive;
    m_leftDistance = leftDistance;
    m_rightDistance = rightDistance;
    m_maxSpeed = maxSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoderPositions();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed;
    double rightSpeed;
    if (m_leftDistance > m_rightDistance) {
      leftSpeed = m_maxSpeed;
      rightSpeed = (m_rightDistance / m_leftDistance) * m_maxSpeed;
    } else {
      rightSpeed = m_maxSpeed;
      leftSpeed = (m_leftDistance / m_rightDistance) * m_maxSpeed;
    }
    m_drive.basicDriveLimited(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.basicDriveLimited(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_leftDistance) > Math.abs(m_drive.getLeftPosition())
        || Math.abs(m_rightDistance) > Math.abs(m_drive.getRightPosition());
  }
}
