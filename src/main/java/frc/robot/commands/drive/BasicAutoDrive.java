/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class BasicAutoDrive extends CommandBase {

  private final Drive m_drive;
  private final DoubleSupplier m_leftDistanceSupplier;
  private final DoubleSupplier m_rightDistanceSupplier;
  private final double m_maxSpeed;

  private double m_leftDistance;
  private double m_rightDistance;
  private double m_leftSpeed;
  private double m_rightSpeed;

  private final DoublePreferenceConstant m_syncP;

  private int m_numLoops;

  public BasicAutoDrive(final Drive drive, final DoubleSupplier leftDistance, final DoubleSupplier rightDistance, final double maxSpeed) {
    m_drive = drive;
    m_leftDistanceSupplier = leftDistance;
    m_rightDistanceSupplier = rightDistance;
    m_maxSpeed = maxSpeed;
    m_syncP = new DoublePreferenceConstant("Basic Auto Drive kP", 0.2);

    addRequirements(m_drive);
  }

  /**
   * Creates a new BasicAutoDrive.
   */
  public BasicAutoDrive(final Drive drive, final double leftDistance, final double rightDistance, final double maxSpeed) {
    this(drive, () -> leftDistance, () -> rightDistance, maxSpeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftDistance = m_leftDistanceSupplier.getAsDouble();
    m_rightDistance = m_rightDistanceSupplier.getAsDouble();

    if (m_leftDistance > m_rightDistance) {
      m_leftSpeed = m_maxSpeed;
      m_rightSpeed = (m_rightDistance / m_leftDistance) * m_maxSpeed;
    } else {
      m_rightSpeed = m_maxSpeed;
      m_leftSpeed = (m_leftDistance / m_rightDistance) * m_maxSpeed;
    }

    m_leftSpeed *= Math.signum(m_leftDistance);
    m_rightSpeed *= Math.signum(m_rightDistance);

    m_drive.resetEncoderPositions();
    m_numLoops = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_numLoops++;

    double desyncAmount = (m_drive.getRightPosition() / (m_rightSpeed / m_maxSpeed)) 
        - (m_drive.getLeftPosition() / (m_leftSpeed / m_maxSpeed));
    m_leftSpeed -= desyncAmount * m_syncP.getValue();
    m_rightSpeed += desyncAmount * m_syncP.getValue();

    m_drive.basicDriveLimited(m_leftSpeed, m_rightSpeed);
    if (m_drive.autoshift((m_leftSpeed + m_rightSpeed) / 2)) {
      m_drive.shiftToHigh();
    } else {
      m_drive.shiftToLow();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_numLoops > 5 // The encoders don't reseting immediately, causing the command to end early without this
        && Math.abs(m_drive.getLeftPosition() / m_leftDistance) + Math.abs(m_drive.getRightPosition() / m_rightDistance) > 2;
  }
}
