/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.driveutil.DriveConfiguration;
import frc.robot.driveutil.DriveUtils;
import frc.robot.driveutil.TJDriveModule;
import frc.robot.util.transmission.CTREMagEncoder;
import frc.robot.util.transmission.Falcon500;
import frc.robot.util.transmission.ShiftingTransmission;

public class Drive extends SubsystemBase {
  /**
   * Creates a new Drive.
   */
  private TJDriveModule m_leftDrive, m_rightDrive;
  private ShiftingTransmission m_leftTransmission, m_rightTransmission;
  private DriveConfiguration m_driveConfiguration;
  private DoubleSolenoid m_leftShifter, m_rightShifter;

  private double m_currentLimit = Constants.DRIVE_CURRENT_LIMIT;
  private double m_maxSpeed;
  private double m_lastLimitSpeed = 0;
  private boolean m_deccelerating = false;

  public Drive() {
    m_driveConfiguration = new DriveConfiguration();

    m_leftTransmission = new ShiftingTransmission(new Falcon500(), Constants.NUM_DRIVE_MOTORS_PER_SIDE,
        new CTREMagEncoder(), Constants.LOW_DRIVE_RATIO, Constants.HIGH_DRIVE_RATIO, Constants.DRIVE_SENSOR_RATIO,
        Constants.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, Constants.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
        Constants.DRIVE_LEFT_LOW_EFFICIENCY, Constants.DRIVE_LEFT_HIGH_EFFICIENCY);
    m_rightTransmission = new ShiftingTransmission(new Falcon500(), Constants.NUM_DRIVE_MOTORS_PER_SIDE,
        new CTREMagEncoder(), Constants.LOW_DRIVE_RATIO, Constants.HIGH_DRIVE_RATIO, Constants.DRIVE_SENSOR_RATIO,
        Constants.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, Constants.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
        Constants.DRIVE_RIGHT_LOW_EFFICIENCY, Constants.DRIVE_RIGHT_HIGH_EFFICIENCY);

    m_leftDrive = new TJDriveModule(m_driveConfiguration.left, m_leftTransmission);
    m_rightDrive = new TJDriveModule(m_driveConfiguration.right, m_rightTransmission);

    m_leftShifter = new DoubleSolenoid(Constants.SHIFTER_LEFT_PCM, Constants.SHIFTER_LEFT_OUT,
        Constants.SHIFTER_LEFT_IN);
    m_rightShifter = new DoubleSolenoid(Constants.SHIFTER_RIGHT_PCM, Constants.SHIFTER_RIGHT_OUT,
        Constants.SHIFTER_RIGHT_IN);

    shiftToLow();
  }

  public void basicDrive(double leftSpeed, double rightSpeed) {
    m_leftDrive.set(ControlMode.PercentOutput, leftSpeed);
    m_rightDrive.set(ControlMode.PercentOutput, rightSpeed);
  }

  /**
   * Commands the drivetrain to the given velocities (in fps) while proactively
   * limiting current draw.
   */
  public void basicDriveLimited(double leftVelocity, double rightVelocity) {
    m_leftDrive.setVelocityCurrentLimited(leftVelocity, m_currentLimit / 2);
    m_rightDrive.setVelocityCurrentLimited(rightVelocity, m_currentLimit / 2);
  }

  /**
   * Arcade drive function for teleop control.
   * 
   * Parameters:
   * 
   * @param speed    The forwards/backwards speed on a scale from -1 to 1
   * @param turnRate The rate to turn at on a scale from -1 (counterclockwise) to
   *                 1 (clockwise)
   */
  public void arcadeDrive(double speed, double turn) {
    speed *= m_maxSpeed;
    turn *= m_maxSpeed;

    DriveUtils.signedPow(speed, Constants.DRIVE_SPEED_EXP);
    DriveUtils.deadbandExponential(turn, Constants.DRIVE_TURN_EXP, 2);

    speed = limitAcceleration(speed);

    double leftSpeed = (speed + turn);
    double rightSpeed = (speed - turn);

    basicDriveLimited(leftSpeed, rightSpeed);
  }

  public double limitAcceleration(double speed) {

    double maxAccel;
    if (isInHighGear()) {
      // if (Robot.m_arm.getDistanceFromBase() >= Constants.ARM_TIPPY_DISTANCE) {
      // maxAccel = Constants.MAX_ACCEL_HIGH_TIPPY;
      // } else {
      maxAccel = Constants.MAX_ACCEL_HIGH;
      // }
    } else {
      // if (Robot.m_arm.getDistanceFromBase() >= Constants.ARM_TIPPY_DISTANCE) {
      // maxAccel = Constants.MAX_ACCEL_LOW_TIPPY;
      // } else {
      maxAccel = Constants.MAX_ACCEL_LOW;
      // }
    }

    double currentSpeed = getStraightSpeed();
    if (speed - currentSpeed > 0) {

      m_deccelerating = false;

      double vel = currentSpeed + maxAccel;
      if (speed < vel) {
        return speed;
      } else {
        return vel;
      }
    } else {

      double vel = getStraightSpeed() - maxAccel;

      if (!m_deccelerating) {
        m_lastLimitSpeed = currentSpeed;
        m_deccelerating = true;
      }

      if (speed > vel) {
        m_lastLimitSpeed = speed;
        return speed;
      } else {
        if (vel > m_lastLimitSpeed) {
          return m_lastLimitSpeed;
        } else {
          m_lastLimitSpeed = vel;
          return vel;
        }
      }

    }

  }

  public void shiftToLow() {
    m_leftShifter.set(Value.kForward);
    m_rightShifter.set(Value.kForward);

    m_leftTransmission.shiftToLow();
    m_rightTransmission.shiftToLow();

    m_maxSpeed = Constants.MAX_SPEED_LOW;
  }

  public void shiftToHigh() {
    m_leftShifter.set(Value.kReverse);
    m_rightShifter.set(Value.kReverse);

    m_leftTransmission.shiftToHigh();
    m_rightTransmission.shiftToHigh();

    m_maxSpeed = Constants.MAX_SPEED_HIGH;
  }

  public boolean isInHighGear() {
    return m_leftTransmission.isInHighGear();
  }

  public double getLeftPosition() {
    return m_leftDrive.getScaledSensorPosition();
  }

  public double getRightPosition() {
    return m_rightDrive.getScaledSensorPosition();
  }

  public double getLeftSpeed() {
    return m_leftDrive.getScaledSensorVelocity();
  }

  public double getRightSpeed() {
    return m_rightDrive.getScaledSensorVelocity();
  }

  public double getStraightSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
