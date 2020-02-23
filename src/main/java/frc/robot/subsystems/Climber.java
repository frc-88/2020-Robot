/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Climber extends SubsystemBase{
    private TalonFX m_climber_motor_left = new TalonFX(Constants.CLIMBER_MOTOR_LEFT);
    private TalonFX m_climber_motor_right = new TalonFX(Constants.CLIMBER_MOTOR_RIGHT);

    private DoubleSolenoid m_climber_ratchet = new DoubleSolenoid(Constants.CLIMBER_PNEUMATICS_FORWARD, Constants.CLIMBER_PNEUMATICS_REVERSE);
    private DoublePreferenceConstant right_kP;
    private DoublePreferenceConstant right_kI;
    private DoublePreferenceConstant right_kD;
    private DoublePreferenceConstant right_kF;
    private DoublePreferenceConstant left_kP;
    private DoublePreferenceConstant left_kI;
    private DoublePreferenceConstant left_kD;
    private DoublePreferenceConstant left_kF;

    public Climber() {
        m_climber_motor_left.configFactoryDefault();
        m_climber_motor_right.configFactoryDefault();
        m_climber_motor_left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climber_motor_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climber_motor_left.setInverted(true);
        m_climber_motor_right.setInverted(false);

        exitZeroMode();

        m_climber_motor_left.configForwardSoftLimitEnable(true);
        m_climber_motor_left.configForwardSoftLimitThreshold(Constants.CLIMBER_MAX_POSITION);
        m_climber_motor_left.configReverseSoftLimitEnable(true);
        m_climber_motor_left.configReverseSoftLimitThreshold(Constants.CLIMBER_MIN_POSITION);
        
        m_climber_motor_right.configForwardSoftLimitEnable(true);
        m_climber_motor_right.configForwardSoftLimitThreshold(Constants.CLIMBER_MAX_POSITION);
        m_climber_motor_right.configReverseSoftLimitEnable(true);
        m_climber_motor_right.configReverseSoftLimitThreshold(Constants.CLIMBER_MIN_POSITION);

        //right pid
        right_kP = new DoublePreferenceConstant("Climber Right kP", 0);
        right_kP.addChangeHandler((Double kP) -> m_climber_motor_right.config_kP(0, kP));
        right_kI = new DoublePreferenceConstant("Climber Right kI", 0);
        right_kI.addChangeHandler((Double kI) -> m_climber_motor_right.config_kP(0, kI));
        right_kD = new DoublePreferenceConstant("Climber Right kD", 0);
        right_kD.addChangeHandler((Double kD) -> m_climber_motor_right.config_kP(0, kD));
        right_kF = new DoublePreferenceConstant("Climber Right kF", 0);
        right_kF.addChangeHandler((Double kF) -> m_climber_motor_right.config_kP(0, kF));

        //left pid
        left_kP = new DoublePreferenceConstant("Climber Left kP", 0);
        left_kP.addChangeHandler((Double kP) -> m_climber_motor_left.config_kP(0, kP));
        left_kI = new DoublePreferenceConstant("Climber Left kI", 0);
        left_kI.addChangeHandler((Double kI) -> m_climber_motor_left.config_kP(0, kI));
        left_kD = new DoublePreferenceConstant("Climber Left kD", 0);
        left_kD.addChangeHandler((Double kD) -> m_climber_motor_left.config_kP(0, kD));
        left_kF = new DoublePreferenceConstant("Climber Left kF", 0);
        left_kF.addChangeHandler((Double kF) -> m_climber_motor_left.config_kP(0, kF));
        
    }

    public void setMotors(final double speed) {
        m_climber_motor_left.set(ControlMode.PercentOutput, speed);
        m_climber_motor_right.set(ControlMode.PercentOutput, speed);
    }

    public void setLeftMotor(final double speed) {
        m_climber_motor_left.set(ControlMode.PercentOutput, speed);
    }

    public void setRightMotor(final double speed) {
        m_climber_motor_right.set(ControlMode.PercentOutput, speed);
    }

    public void setBothMotorHeights(final double leftHeight, final double rightHeight) {
        m_climber_motor_left.set(ControlMode.Position, heightToMotorTicks(leftHeight));
        m_climber_motor_right.set(ControlMode.Position, heightToMotorTicks(rightHeight));
    }

    public double getLeftPosition() {
        return motorTicksToHeight(m_climber_motor_left.getSelectedSensorPosition());
    }

    public double getRightPosition() {
        return motorTicksToHeight(m_climber_motor_right.getSelectedSensorPosition());
    }

    public void setLeftMotorHeight(final double leftHeight) {
        m_climber_motor_left.set(ControlMode.Position, heightToMotorTicks(leftHeight));
    }

    public void setRightMotorHeight(final double rightHeight) {
        m_climber_motor_right.set(ControlMode.Position, heightToMotorTicks(rightHeight));
    }

    public void setPositionChange(final double leftSpeed, final double rightSpeed) {
        setBothMotorHeights(getLeftPosition() + (leftSpeed * Constants.CLIMBER_MANUAL_CONTROL_SCALAR), getRightPosition() + (rightSpeed * Constants.CLIMBER_MANUAL_CONTROL_SCALAR));
    }

    public void engageRatchets() {
        m_climber_motor_left.configPeakOutputForward(0);
        m_climber_motor_right.configPeakOutputForward(0);
        m_climber_ratchet.set(Value.kReverse);
    }

    public void disengageRatchets() {
        m_climber_motor_left.configPeakOutputForward(1);
        m_climber_motor_right.configPeakOutputForward(1);
        m_climber_ratchet.set(Value.kForward);
    }

    public void zero() {
        m_climber_motor_left.setSelectedSensorPosition(0);
        m_climber_motor_right.setSelectedSensorPosition(0);
    }

    public void enterZeroMode() {
        StatorCurrentLimitConfiguration currentLimit = new StatorCurrentLimitConfiguration();
        currentLimit.enable = true;
        currentLimit.currentLimit = 10;
        currentLimit.triggerThresholdCurrent = 10;
        currentLimit.triggerThresholdTime = 0.001;
        m_climber_motor_left.configStatorCurrentLimit(currentLimit);
        m_climber_motor_right.configStatorCurrentLimit(currentLimit);  
        m_climber_motor_left.configReverseSoftLimitEnable(false);   
        m_climber_motor_right.configReverseSoftLimitEnable(false);   
    }

    public void exitZeroMode() {
        StatorCurrentLimitConfiguration currentLimit = new StatorCurrentLimitConfiguration();
        currentLimit.enable = true;
        currentLimit.currentLimit = 40;
        currentLimit.triggerThresholdCurrent = 40;
        currentLimit.triggerThresholdTime = 0.001;
        m_climber_motor_left.configStatorCurrentLimit(currentLimit);
        m_climber_motor_right.configStatorCurrentLimit(currentLimit);   
        m_climber_motor_left.configReverseSoftLimitEnable(true);   
        m_climber_motor_right.configReverseSoftLimitEnable(true); 
    }

    public double motorTicksToHeight(int ticks) {
        return (ticks * (1. / 2048.) * (2. * Math.PI) * (1. / Constants.CLIMBER_GEAR_RATIO));
    }

    public int heightToMotorTicks(double height) {
        return (int)(height * 2048. * (1. / (2. * Math.PI)) * Constants.CLIMBER_GEAR_RATIO);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climber Position", m_climber_motor_left.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Climber Position", m_climber_motor_right.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Climber Velocity", m_climber_motor_left.getSelectedSensorVelocity() / 10);
        SmartDashboard.putNumber("Right Climber Velocity", m_climber_motor_right.getSelectedSensorVelocity() / 10);
        SmartDashboard.putNumber("Left Climber Stator", m_climber_motor_left.getStatorCurrent());
        SmartDashboard.putNumber("Right Climber Stator", m_climber_motor_right.getStatorCurrent());
        SmartDashboard.putNumber("Left Climber Supply", m_climber_motor_left.getSupplyCurrent());
        SmartDashboard.putNumber("Right Climber Supply", m_climber_motor_right.getSupplyCurrent());

        SmartDashboard.putNumber("Right climber output voltage", m_climber_motor_right.getMotorOutputVoltage());
        SmartDashboard.putNumber("Left climber output voltage", m_climber_motor_left.getMotorOutputVoltage());
    }
}
