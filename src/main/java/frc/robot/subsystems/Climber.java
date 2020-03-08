/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//I wasn't able to get a lot of information on the climber, some of this information might not be correct
//or might not reflect the abilities of the climber.

public class Climber extends SubsystemBase{
    private TalonFX m_climber_motor_left = new TalonFX(Constants.CLIMBER_MOTOR_LEFT);
    private TalonFX m_climber_motor_right = new TalonFX(Constants.CLIMBER_MOTOR_RIGHT);

    private DoubleSolenoid m_climber_ratchet = new DoubleSolenoid(Constants.CLIMBER_PNEUMATICS_FORWARD, Constants.CLIMBER_PNEUMATICS_REVERSE);

    public Climber() {
        m_climber_motor_left.configFactoryDefault();
        m_climber_motor_right.configFactoryDefault();
        m_climber_motor_left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climber_motor_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climber_motor_left.setInverted(true);
        m_climber_motor_right.setInverted(false);
        m_climber_motor_left.setNeutralMode(NeutralMode.Brake);
        m_climber_motor_right.setNeutralMode(NeutralMode.Brake);

        exitZeroMode();

        m_climber_motor_left.configForwardSoftLimitEnable(true);
        m_climber_motor_left.configForwardSoftLimitThreshold(Constants.CLIMBER_MAX_POSITION);
        m_climber_motor_left.configReverseSoftLimitEnable(true);
        m_climber_motor_left.configReverseSoftLimitThreshold(Constants.CLIMBER_MIN_POSITION);
        
        m_climber_motor_right.configForwardSoftLimitEnable(true);
        m_climber_motor_right.configForwardSoftLimitThreshold(Constants.CLIMBER_MAX_POSITION);
        m_climber_motor_right.configReverseSoftLimitEnable(true);
        m_climber_motor_right.configReverseSoftLimitThreshold(Constants.CLIMBER_MIN_POSITION);

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

    public int getLeftMotorPosition() {
        return m_climber_motor_left.getSelectedSensorPosition();
    }

    public int getRightMotorPosition() {
        return m_climber_motor_right.getSelectedSensorPosition();
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
        currentLimit.currentLimit = 50;
        currentLimit.triggerThresholdCurrent = 50;
        currentLimit.triggerThresholdTime = 0.001;
        m_climber_motor_left.configStatorCurrentLimit(currentLimit);
        m_climber_motor_right.configStatorCurrentLimit(currentLimit);   
        m_climber_motor_left.configReverseSoftLimitEnable(true);   
        m_climber_motor_right.configReverseSoftLimitEnable(true); 
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
