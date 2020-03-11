/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sensors;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sensors;

public class WaitForBallsShot extends CommandBase {

  private final Sensors m_sensors;
  private final int m_numBalls;

  private boolean m_currentSensorState;
  private int m_numDebounceTicks;
  private int m_numBallsPassed;

  private static final int DEBOUNCE_TICKS = 2;

  /**
   * Creates a new WaitForBalls.
   */
  public WaitForBallsShot(final Sensors sensors, final int numBalls) {
    m_sensors = sensors;
    m_numBalls = numBalls;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentSensorState = false;
    m_numDebounceTicks = 0;
    m_numBallsPassed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_sensors.hasBallInShooter() != m_currentSensorState) {
      m_numDebounceTicks++;
    } else {
      m_numDebounceTicks = 0;
    }

    if (m_numDebounceTicks >= DEBOUNCE_TICKS) {
      m_numDebounceTicks = 0;
      m_currentSensorState = !m_currentSensorState;
      if (!m_currentSensorState) {
        m_numBallsPassed++;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_numBallsPassed >= m_numBalls;
  }
}
