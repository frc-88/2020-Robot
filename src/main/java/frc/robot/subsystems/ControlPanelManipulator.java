/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import java.util.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;


public class ControlPanelManipulator extends SubsystemBase {
  /**
   * Creates a new ControlPanelManipulator.
   */
  //adjusted color rgb values based on testing
  private final Color kBlueTarget = ColorMatch.makeColor(0.12, 0.41, 0.46);
  private final Color kGreenTarget = ColorMatch.makeColor(0.17, 0.55, 0.26);
  private final Color kRedTarget = ColorMatch.makeColor(0.54, 0.33, 0.12);
  private final Color kYellowTarget = ColorMatch.makeColor(0.31, 0.56, 0.12);
  private final float controlPanelSlice = 12.5f;
  private final double motorRotationsPerWheelRotation = (100. / (2. * Math.PI));
  private final int ticksPerMotorRotation = 2048;

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private TalonFX m_spinner = new TalonFX(Constants.CPM_MOTOR);
  private final I2C.Port m_i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(m_i2cPort);
  private Encoder m_wristEncoder = new Encoder(Constants.CPM_JOINT_ENCODER_CHANNEL_1A, Constants.CPM_JOINT_ENCODER_CHANNEL_1B);
  private DoubleSolenoid m_pneumatics = new DoubleSolenoid(Constants.CPM_PNEUMATICS_FORWARD, Constants.CPM_PNEUMATICS_REVERSE);
  private AHRS m_navX = new AHRS(SPI.Port.kMXP); 
  private DigitalInput m_contactSensor = new DigitalInput(Constants.CPM_DIGITAL_INPUT_CHANNEL);

  private double MAXIMUM_WHEEL_VELOCITY = (59. / 60.);
  private DoublePreferenceConstant MAXIMUM_WHEEL_ACCELERATION;
  private DoublePreferenceConstant spinner_kP;
  private DoublePreferenceConstant spinner_kI;
  private DoublePreferenceConstant spinner_kD;
  private DoublePreferenceConstant spinner_kF;
  

  public ControlPanelManipulator() {
    MAXIMUM_WHEEL_ACCELERATION = new DoublePreferenceConstant("CPM Acceleration", 2);
    MAXIMUM_WHEEL_ACCELERATION.addChangeHandler(
      (Double acceleration) -> m_spinner.configMotionAcceleration(convertWheelVelocityToMotorVelocity(acceleration)));
    spinner_kP = new DoublePreferenceConstant("CPM Spinner kP", 0);
    spinner_kP.addChangeHandler((Double kP) -> m_spinner.config_kP(0, kP));
    spinner_kI = new DoublePreferenceConstant("CPM Spinner kI", 0);
    spinner_kI.addChangeHandler((Double kI) -> m_spinner.config_kI(0, kI));
    spinner_kD = new DoublePreferenceConstant("CPM Spinner kD", 0);
    spinner_kD.addChangeHandler((Double kD) -> m_spinner.config_kD(0, kD));
    spinner_kF = new DoublePreferenceConstant("CPM Spinner kF", 0);
    spinner_kF.addChangeHandler((Double kF) -> m_spinner.config_kF(0, kF));

    m_wristEncoder.reset();
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_spinner.configFactoryDefault();
    m_spinner.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_spinner.configMotionCruiseVelocity(convertWheelVelocityToMotorVelocity(MAXIMUM_WHEEL_VELOCITY));
    m_spinner.configMotionAcceleration(convertWheelVelocityToMotorVelocity(MAXIMUM_WHEEL_ACCELERATION.getValue()));
    m_spinner.selectProfileSlot(0, 0);

    SmartDashboard.putBoolean("Zero CPM", false);
  }

  public boolean isEngaged() {
    return m_contactSensor.get();
  }

  public String getColor() {
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "b";
    } else if (match.color == kRedTarget) {
      colorString = "r";
    } else if (match.color == kGreenTarget) {
      colorString = "g";
    } else if (match.color == kYellowTarget) {
      colorString = "y";
    } else {
      colorString = "?";
    }
    return colorString;
  }

  public String getFMSColorTarget() {
    return DriverStation.getInstance().getGameSpecificMessage();
  }

  public double getRobotFacing() {
    return m_navX.getYaw();
  }

  public double getRobotWrist() {
    return m_wristEncoder.get()/Constants.CPM_WRIST_ENCODER_COUNTS_PER_REV*360;
  }

  public float calcPositionControlTargetPosition() {
    //calculate in inches, motor can spin x inches
    int colorDistance = 0;
    int i = 1; //Start at index 1
    String currentDetectedColor = getColor();
    String currentTargetColor = getFMSColorTarget();
    String[] colorList = { "r", "g", "b", "y", "r", "g"};
    int colorListLength = colorList.length;
    while (i < colorListLength) {
      if (colorList[i] == currentDetectedColor) {
        if (colorList[i-1] == currentTargetColor) {
          colorDistance = -1;
          break;
        }
        else if (colorList[i+1] == currentTargetColor) {
          colorDistance = 1;
          break;
        }
        else {
          colorDistance = 2;
          break;
        }
      }
    }
    return colorDistance*12.5f;
  }

  public Color getRawColor() {
    Color detectedColor = m_colorSensor.getColor();
    return detectedColor;
  }

  /**
   * Converts motor position to wheel position
   * 
   * @param motorPosition The motor position, in ticks
   * @return The wheel position, in rotations
   */
  private double convertMotorPositionToWheelPosition(int motorPosition) {
    return (motorPosition * (1. / ticksPerMotorRotation) * (1. / motorRotationsPerWheelRotation));
  }

  /**
   * Converts wheel position to motor position
   * 
   * @param wheelPosition The wheel position, in rotations
   * @return The motor position, in ticks
   */
  private int convertWheelPositionToMotorPosition(double wheelPosition) {
    return (int)(wheelPosition * motorRotationsPerWheelRotation * ticksPerMotorRotation);
  }

  /**
   * Converts wheel velocity to motor velocity
   * 
   * @param wheelVelocity The wheel velocity, in rotations per second
   * @return The motor velocity, in ticks per 100ms
   */
  private int convertWheelVelocityToMotorVelocity(double wheelVelocity) {
    return (int)(wheelVelocity * (1. / 10.) * motorRotationsPerWheelRotation * ticksPerMotorRotation);
  }

  /**
   * Converts motor velocity to wheel velocity
   * 
   * @param motorVelocity The motor velocity, in ticks per 100ms
   * @return The wheel velocity, in rotations per second
   */
  private double convertMotorVelocityToWheelVelocity(int motorVelocity) {
    return (motorVelocity * (1. / ticksPerMotorRotation) * (1. / motorRotationsPerWheelRotation) * 10.);
  }

  public double getWheelPosition() {
    return convertMotorPositionToWheelPosition(m_spinner.getSelectedSensorPosition());
  }

  public double getWheelVelocity() {
    return convertMotorVelocityToWheelVelocity(m_spinner.getSelectedSensorVelocity());
  }

  public void setWheelPosition(double position) {
    m_spinner.setSelectedSensorPosition(convertWheelPositionToMotorPosition(position));
  }

  public void moveWheelToPosition(double wheelPosition) {
    m_spinner.set(TalonFXControlMode.MotionMagic, convertWheelPositionToMotorPosition(wheelPosition));
    //System.out.println(convertWheelPositionToMotorPosition(wheelPosition));
    System.out.println(m_spinner.getControlMode());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CPM Velocity", getWheelVelocity());
    SmartDashboard.putNumber("CPM Position", getWheelPosition());
    SmartDashboard.putNumber("CPM Target Position", convertMotorPositionToWheelPosition(m_spinner.getActiveTrajectoryPosition()));
    SmartDashboard.putNumber("CPM Target Velocity", convertMotorVelocityToWheelVelocity(m_spinner.getActiveTrajectoryVelocity()));

    SmartDashboard.putString("robot sensor color", getColor());
    SmartDashboard.putNumber("Red", getRawColor().red);
    SmartDashboard.putNumber("Green", getRawColor().green);
    SmartDashboard.putNumber("Blue", getRawColor().blue);

    if(SmartDashboard.getBoolean("Zero CPM", false)) {
      setWheelPosition(0);
    }

    m_spinner.config_kP(0, spinner_kP.getValue());
    m_spinner.config_kI(0, spinner_kI.getValue());
    m_spinner.config_kD(0, spinner_kD.getValue());
    m_spinner.config_kF(0, spinner_kF.getValue());
  }
}
