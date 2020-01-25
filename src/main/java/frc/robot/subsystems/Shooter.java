/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX m_shooterMotor1 = new TalonFX(Constants.SHOOTER_MOTOR);
  private TalonFX m_shooterMotor2 = new TalonFX(Constants.SHOOTER_MOTOR_2);
  private TalonFX m_feederMotor = new TalonFX(Constants.SHOOTER_FEEDER_MOTOR);
  private TalonFX m_rotatorMotor = new TalonFX(Constants.SHOOTER_ROTATOR_MOTOR);
  
  private Encoder m_absoluteArmAngleEncoder = new Encoder(Constants.SHOOTER_ANGLE_ENCODER_CHANNEL_1A, Constants.SHOOTER_ANGLE_ENCODER_CHANNEL_1B);
  private TalonFXSensorCollection m_shooterMotor1Sensor = new TalonFXSensorCollection(m_shooterMotor1);
  private TalonFXSensorCollection m_shooterMotor2Sensor = new TalonFXSensorCollection(m_shooterMotor2);
  private TalonFXSensorCollection m_feederMotorSensor = new TalonFXSensorCollection(m_feederMotor);
  private TalonFXSensorCollection m_rotatorMotorSensor = new TalonFXSensorCollection(m_rotatorMotor);

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
