/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 *   intake be like "eat!"
 * they eating them oh my gosh
 *     robot hungy lol
 */

public class Intake extends SubsystemBase {

  private TalonSRX rollerMotor;
  private DoubleSolenoid deployPiston;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    rollerMotor = new TalonSRX(Constants.ROLLER_ID);

    rollerMotor.configFactoryDefault();
    SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration();
    currentLimit.enable = true;
    currentLimit.triggerThresholdTime = 0.250;
    currentLimit.triggerThresholdCurrent = 55;
    currentLimit.currentLimit = 35;
    rollerMotor.configSupplyCurrentLimit(currentLimit);
    

    deployPiston = new DoubleSolenoid(Constants.INTAKE_DEPLOY_PISTON, Constants.INTAKE_RETRACT_PISTON);
  }

  public void setRoller(double speed) {
    rollerMotor.set(ControlMode.PercentOutput, speed);
}


  public void deploy() {
    deployPiston.set(Value.kForward);
  }

  public void retract(){
    deployPiston.set(Value.kReverse);
  }

  public boolean isDeployed() {
    return deployPiston.get() == Value.kForward;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake stator current", rollerMotor.getStatorCurrent());
    SmartDashboard.putNumber("Intake supply current", rollerMotor.getSupplyCurrent());
    // This method will be called once per scheduler run
  }
}