/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ArmConfig;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */

  private double speed;

  private TalonFX m_rotator = new TalonFX(Constants.ARM_MOTOR);

  private ArmConfig armConfig;

  public Arm() {
    m_rotator.configFactoryDefault();
    m_rotator.configAllSettings(armConfig.armConfiguration);

    m_rotator.enableVoltageCompensation(true);
    m_rotator.setInverted(false);
    m_rotator.setSensorPhase(false);
    m_rotator.setNeutralMode(NeutralMode.Brake);
  }

  private void rotateArm(double speed) {
    m_rotator.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
