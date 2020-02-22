/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import frc.robot.util.CPMConfig;
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

  // set constants for how the motor motion relates to movement of the color wheel
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

  // because we can't move the color wheel more than 1 rotation per second 
  private double MAXIMUM_WHEEL_VELOCITY = (59. / 60.); // degrees/second  - just under 1 rotation per second
  private double TIME_TO_REACH_MAX_VELOCITY = .5; // get to max speed in 1/2 a second
  private double INIT_MAX_WHEEL_ACCELERATION = MAXIMUM_WHEEL_VELOCITY/ TIME_TO_REACH_MAX_VELOCITY; // =~ 2
  private DoublePreferenceConstant MAXIMUM_WHEEL_ACCELERATION;
  private double COLOR_WHEEL_SENSOR_SLICE_OFFSET = 2.;
  
  private DoublePreferenceConstant spinner_kP;
  private DoublePreferenceConstant spinner_kI;
  private DoublePreferenceConstant spinner_kD;
  private DoublePreferenceConstant spinner_kF;
  
  private CPMConfig cpmConfig;

  
  public ControlPanelManipulator() {
    m_wristEncoder.reset();

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    m_spinner.configFactoryDefault();
    //m_spinner.configAllSettings(cpmConfig.cpmConfiguration);
    m_spinner.enableVoltageCompensation(true);
    m_spinner.setSensorPhase(false);
    m_spinner.setInverted(false);
    m_spinner.setNeutralMode(NeutralMode.Brake);

    /* Motion Magic */
    m_spinner.selectProfileSlot(0, 0);
    MAXIMUM_WHEEL_ACCELERATION = new DoublePreferenceConstant("CPM Acceleration", INIT_MAX_WHEEL_ACCELERATION);
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

    m_spinner.configMotionCruiseVelocity(convertWheelVelocityToMotorVelocity(MAXIMUM_WHEEL_VELOCITY));
    m_spinner.configMotionAcceleration(convertWheelVelocityToMotorVelocity(MAXIMUM_WHEEL_ACCELERATION.getValue()));
  
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
    return DriverStation.getInstance().getGameSpecificMessage().toLowerCase();
  }

  public double getRobotFacing() {
    return m_navX.getYaw();
  }

  public double getRobotWrist() {
    return m_wristEncoder.get()/Constants.CPM_WRIST_ENCODER_COUNTS_PER_REV*360;
  }

  public double calcPositionControlTargetPosition() {
    //calculate in inches, motor can spin x inches
    double colorDistance = 0;
    final int i = 0; //Start at index 1
    final String currentRobotDetectedColor = getColor();
    final String currentTargetColor = getFMSColorTarget();
    final String colorListString = "yrgb"; //string for what half the wheel looks like
    final int robotSensorColorPosition = colorListString.indexOf(getColor()); // find position in string
    final int gameSensorColorPosition = colorListString.indexOf(getFMSColorTarget()); // find position in string

    int positionDistance = robotSensorColorPosition - gameSensorColorPosition;
    int slicesToMove = 0; // needs to account for the two slice offset between robot and game sensor
    int movementDirection = 1;
    
    if (positionDistance<0){ // reduces the number of cases in the switch statement
      movementDirection = -1;
    } 
    
    switch(Math.abs(positionDistance)) {
      // identify how many slices to move but also account for the 2 slice offset between sensors
      case 0:
        slicesToMove = 2; 
        break;
      case 1:
        slicesToMove = 1 * movementDirection;
        break;
      case 2:
        slicesToMove = 0;
        break;
      case 3:
        slicesToMove = -1 * movementDirection;
    }
    /*
    while (i < colorList.length) {
      if (colorList[i] == currentRobotDetectedColor) {
        if (colorList[i-1] == currentTargetColor) {
          colorDistance = -1;
          System.out.println("\nCPM: The color distance is equal to -1");
          break;
        }
        else if (colorList[i+1] == currentTargetColor) {
          colorDistance = 1;
          System.out.println("\nCPM: The color distance is equal to 1");
          break;
        }
        else if (colorList[i+2] == currentTargetColor) {
          colorDistance = 2;
          System.out.println("\nCPM: The color distance is equal to 2");
          break;
        }
        else {
          colorDistance = 0;
          System.out.println("\nCPM: The color distance is equal to 0");
          break;
        }
      }
    }*/

    return this.convertWheelPositionToMotorPosition(slicesToMove/Constants.CPM_NUM_SLICES);
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
   * 
   * @return The motor position from the integrated sensor in ticks
   */
  public double getMotorSensorPosition(){
    return m_spinner.getSelectedSensorPosition();
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

  public void setMotorSensorPosition(int sensorPosition){
    m_spinner.setSelectedSensorPosition(sensorPosition);
  }

  public void moveWheelToPosition(double wheelPosition) {
    //System.out.println("CPM: convert wheel postion to motor postion"+convertWheelPositionToMotorPosition(wheelPosition));
    //SmartDashboard.putNumber("CPM W2M Conversion",convertWheelPositionToMotorPosition(wheelPosition));
    //m_spinner.set(ControlMode.Position, convertWheelPositionToMotorPosition(wheelPosition));
    m_spinner.set(ControlMode.MotionMagic, convertWheelPositionToMotorPosition(wheelPosition));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CPM Wheel Velocity", getWheelVelocity());
    SmartDashboard.putNumber("CPM Wheel Position", getWheelPosition());
    SmartDashboard.putNumber("CPM Motor Position", getMotorSensorPosition());
    SmartDashboard.putNumber("CPM Motor Velocity", m_spinner.getActiveTrajectoryVelocity());

    SmartDashboard.putNumber("CPM Target Position", convertMotorPositionToWheelPosition(m_spinner.getActiveTrajectoryPosition()));
    SmartDashboard.putNumber("CPM Target Velocity", convertMotorVelocityToWheelVelocity(m_spinner.getActiveTrajectoryVelocity()));

    SmartDashboard.putString("CPM sensor color", getColor());
    SmartDashboard.putNumber("Red", getRawColor().red);
    SmartDashboard.putNumber("Green", getRawColor().green);
    SmartDashboard.putNumber("Blue", getRawColor().blue);

    SmartDashboard.putBoolean("CPM isEngaged", isEngaged());

    if(SmartDashboard.getBoolean("Zero CPM Sensor", false)) {
      setMotorSensorPosition(0);
      System.out.println("CPM: Setting wheel sensor postion to 0");
    }

    m_spinner.config_kP(0, spinner_kP.getValue());
    m_spinner.config_kI(0, spinner_kI.getValue());
    m_spinner.config_kD(0, spinner_kD.getValue());
    m_spinner.config_kF(0, spinner_kF.getValue());
  }
}
