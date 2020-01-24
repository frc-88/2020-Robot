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

public class Intake extends SubsystemBase {

  private TalonSRX rollerMotor;
  private DoubleSolenoid deployPiston;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    rollerMotor = new TalonSRX(Constants.ROLLER_ID);
    deployPiston = new DoubleSolenoid(Constants.INTAKE_DEPLOY_PISTON, Constants.INTAKE_RETRACT_PISTON);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setRoller(double speed){
    rollerMotor.set(ControlMode.PercentOutput, speed);
  }

  
  public void deploy(){
    deployPiston.set(Value.kForward);
  }


  public void retract(){
    deployPiston.set(Value.kReverse);
  }

}