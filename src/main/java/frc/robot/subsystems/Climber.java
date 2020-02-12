/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//I wasn't able to get a lot of information on the climber, some of this information might not be correct
//or might not reflect the abilities of the climber.

public class Climber extends SubsystemBase{
    private TalonFX m_climber_motor_left = new TalonFX(Constants.CLIMBER_MOTOR_LEFT);
    private TalonFX m_climber_motor_right = new TalonFX(Constants.CLIMBER_MOTOR_RIGHT);

    private DoubleSolenoid m_climber_ratchet_right = new DoubleSolenoid(Constants.CLIMBER_PNEUMATICS_RIGHT_FORWARD, Constants.CLIMBER_PNEUMATICS_RIGHT_REVERSE);
    private DoubleSolenoid m_climber_ratchet_left = new DoubleSolenoid(Constants.CLIMBER_PNEUMATICS_LEFT_FORWARD, Constants.CLIMBER_PNEUMATICS_LEFT_REVERSE);

    public Climber() {
        m_climber_motor_left.configFactoryDefault();
        m_climber_motor_right.configFactoryDefault();
        m_climber_motor_left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climber_motor_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climber_motor_left.setSelectedSensorPosition(0);
        m_climber_motor_right.setSelectedSensorPosition(0);
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

    public void deployRatchets() {
        m_climber_ratchet_left.set(Value.kForward);
        m_climber_ratchet_right.set(Value.kForward);
    }

    public void retractRatchets() {
        m_climber_ratchet_left.set(Value.kReverse);
        m_climber_ratchet_right.set(Value.kReverse);
    }
}
