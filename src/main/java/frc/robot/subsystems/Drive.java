/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.driveutil.DriveConfiguration;
import frc.robot.driveutil.DriveUtils;
import frc.robot.driveutil.TJDriveModule;
import frc.robot.util.SyncPIDController;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;
import frc.robot.util.transmission.CTREMagEncoder;
import frc.robot.util.transmission.Falcon500;
import frc.robot.util.transmission.ShiftingTransmission;

public class Drive extends SubsystemBase {
  /**
   * Creates a new Drive.
   */
  private final TJDriveModule m_leftDrive, m_rightDrive;
  private final CANCoder m_leftEncoder, m_rightEncoder;
  private ShiftingTransmission m_leftTransmission, m_rightTransmission;
  private SyncPIDController m_leftVelPID, m_rightVelPID;
  private DriveConfiguration m_driveConfiguration;
  private DoubleSolenoid m_leftShifter, m_rightShifter;

  private double m_currentLimit = Constants.DRIVE_CURRENT_LIMIT;
  private double m_maxSpeed;
  
  private PIDPreferenceConstants leftVelPIDConstants;
  private PIDPreferenceConstants rightVelPIDConstants;
  private DoublePreferenceConstant downshiftSpeed;
  private DoublePreferenceConstant upshiftSpeed;

  public Drive() {
    m_driveConfiguration = new DriveConfiguration();

    leftVelPIDConstants = new PIDPreferenceConstants("Left Drive Vel", 0, 0.015, 0, 0, 2, 2, 0);
    rightVelPIDConstants = new PIDPreferenceConstants("Right Drive Vel", 0, 0.015, 0, 0, 2, 2, 0);
    downshiftSpeed = new DoublePreferenceConstant("Downshift Speed", 5);
    upshiftSpeed = new DoublePreferenceConstant("UpshiftSpeed", 6);

    m_leftTransmission = new ShiftingTransmission(new Falcon500(), Constants.NUM_DRIVE_MOTORS_PER_SIDE,
        new CTREMagEncoder(), Constants.LOW_DRIVE_RATIO, Constants.HIGH_DRIVE_RATIO, Constants.DRIVE_SENSOR_RATIO,
        Constants.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, Constants.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
        Constants.DRIVE_LEFT_LOW_EFFICIENCY, Constants.DRIVE_LEFT_HIGH_EFFICIENCY);
    m_rightTransmission = new ShiftingTransmission(new Falcon500(), Constants.NUM_DRIVE_MOTORS_PER_SIDE,
        new CTREMagEncoder(), Constants.LOW_DRIVE_RATIO, Constants.HIGH_DRIVE_RATIO, Constants.DRIVE_SENSOR_RATIO,
        Constants.DRIVE_LOW_STATIC_FRICTION_VOLTAGE, Constants.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE,
        Constants.DRIVE_RIGHT_LOW_EFFICIENCY, Constants.DRIVE_RIGHT_HIGH_EFFICIENCY);

    m_leftVelPID = new SyncPIDController(leftVelPIDConstants);
    m_rightVelPID = new SyncPIDController(rightVelPIDConstants);
    m_leftTransmission.setVelocityPID(m_leftVelPID);
    m_rightTransmission.setVelocityPID(m_rightVelPID);

    m_leftEncoder = new CANCoder(Constants.LEFT_DRIVE_ENCODER_ID);
    m_rightEncoder = new CANCoder(Constants.RIGHT_DRIVE_ENCODER_ID);

    m_leftEncoder.configFactoryDefault();
    m_rightEncoder.configFactoryDefault();

    m_leftDrive = new TJDriveModule(m_driveConfiguration.left, m_leftTransmission);
    m_rightDrive = new TJDriveModule(m_driveConfiguration.right, m_rightTransmission);

    m_leftDrive.configRemoteFeedbackFilter(m_leftEncoder, 0);
    m_rightDrive.configRemoteFeedbackFilter(m_rightEncoder, 0);

    m_leftShifter = new DoubleSolenoid(Constants.SHIFTER_LEFT_PCM, Constants.SHIFTER_LEFT_OUT,
        Constants.SHIFTER_LEFT_IN);
    m_rightShifter = new DoubleSolenoid(Constants.SHIFTER_RIGHT_PCM, Constants.SHIFTER_RIGHT_OUT,
        Constants.SHIFTER_RIGHT_IN);

    shiftToLow();

    SmartDashboard.putBoolean("Zero Drive", false);
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
    double leftExpectedCurrent = m_leftDrive.getExpectedCurrentDraw(leftVelocity);
    double rightExpectedCurrent = m_rightDrive.getExpectedCurrentDraw(rightVelocity);
    double totalExpectedCurrent = leftExpectedCurrent + rightExpectedCurrent;
    double leftCurrentLimit = m_currentLimit * leftExpectedCurrent / totalExpectedCurrent;
    double rightCurrentLimit = m_currentLimit * rightExpectedCurrent / totalExpectedCurrent;

    m_leftDrive.setVelocityCurrentLimited(leftVelocity, leftCurrentLimit);
    m_rightDrive.setVelocityCurrentLimited(rightVelocity, rightCurrentLimit);
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

    turn = DriveUtils.cheesyTurn(speed, turn);

    speed *= m_maxSpeed;
    turn *= m_maxSpeed;

    double leftSpeed = (speed + turn);
    double rightSpeed = (speed - turn);

    basicDriveLimited(leftSpeed, rightSpeed);
  }

  public boolean autoshift() {
    double currentSpeed = getStraightSpeed();
    if (isInHighGear() && Math.abs(currentSpeed) <= downshiftSpeed.getValue()) {
      return false;
    } else if (!isInHighGear() && Math.abs(currentSpeed) >= upshiftSpeed.getValue()) {
      return true;
    } else {
      return isInHighGear();
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

  public void setBrakeMode() {
    m_leftDrive.brakeAll();
    m_rightDrive.brakeAll();
  }

  public void setCoastMode() {
    m_leftDrive.coastAll();
    m_rightDrive.coastAll();
  }

  @Override
  public void periodic() {
    if (SmartDashboard.getBoolean("Zero Drive", false)) {
      m_leftEncoder.setPosition(0);
      m_rightEncoder.setPosition(0);
      SmartDashboard.putBoolean("Zero Drive", false);
    }

    SmartDashboard.putNumber("L Drive Current", m_leftDrive.getTotalCurrent());
    SmartDashboard.putNumber("R Drive Current", m_rightDrive.getTotalCurrent());
    SmartDashboard.putNumber("L Drive Speed", m_leftDrive.getScaledSensorVelocity());
    SmartDashboard.putNumber("R Drive Speed", m_rightDrive.getScaledSensorVelocity());
    SmartDashboard.putNumber("L Drive Position", m_leftDrive.getScaledSensorPosition());
    SmartDashboard.putNumber("R Drive Position", m_rightDrive.getScaledSensorPosition());

    if (DriverStation.getInstance().isEnabled()) {
      this.setBrakeMode();
    } else {
      this.setCoastMode();
    }
  }
}
