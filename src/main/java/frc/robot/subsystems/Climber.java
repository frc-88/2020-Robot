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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//I wasn't able to get a lot of information on the climber, some of this information might not be correct
//or might not reflect the abilities of the climber.

public class Climber extends SubsystemBase{
    private TalonFX m_climber_motor1 = new TalonFX(Constants.CLIMBER_MOTOR_LEFT);
    private TalonFX m_climber_motor2 = new TalonFX(Constants.CLIMBER_MOTOR_RIGHT);

    public Climber() {
        m_climber_motor1.configFactoryDefault();
        m_climber_motor2.configFactoryDefault();
        m_climber_motor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climber_motor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climber_motor1.setSelectedSensorPosition(0);
        m_climber_motor2.setSelectedSensorPosition(0);
    }
    public void setMotors(final double speed) {
        m_climber_motor1.set(ControlMode.PercentOutput, speed);
        m_climber_motor2.set(ControlMode.PercentOutput, speed);
    }
}
