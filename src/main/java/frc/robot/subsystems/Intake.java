/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 *   intake be like "eat!"
 * they eating them oh my gosh
 *     robot hungy lol
 */

public class Intake extends SubsystemBase {

  private TalonSRX rollerMotor;
  private DoubleSolenoid leftDeployPiston, rightDeployPiston;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    rollerMotor = new TalonSRX(Constants.ROLLER_ID);
    leftDeployPiston = new DoubleSolenoid(Constants.INTAKE_LEFT_DEPLOY_PISTON, Constants.INTAKE_LEFT_RETRACT_PISTON);
    rightDeployPiston = new DoubleSolenoid(Constants.INTAKE_RIGHT_DEPLOY_PISTON, Constants.INTAKE_RIGHT_RETRACT_PISTON);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setRoller(double speed) {
    if (isDeployed()) {
      rollerMotor.set(ControlMode.PercentOutput, speed);
    } else {
      rollerMotor.set(ControlMode.PercentOutput, 0);
    }
  }


  public void deploy() {
    leftDeployPiston.set(Value.kForward);
    rightDeployPiston.set(Value.kForward);
  }


  public void retract(){
    leftDeployPiston.set(Value.kReverse);
    rightDeployPiston.set(Value.kReverse);
  }

  public boolean isDeployed() {
    return leftDeployPiston.get() == Value.kForward;
  }
}