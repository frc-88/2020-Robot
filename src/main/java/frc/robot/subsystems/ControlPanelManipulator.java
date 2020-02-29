/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;


public class ControlPanelManipulator extends SubsystemBase {
  
  private TalonSRX m_feederSpinner = new TalonSRX(Constants.FEEDER_MOTOR);
  private TalonSRXConfiguration m_config;

  private DoubleSolenoid m_deployer = new DoubleSolenoid(Constants.FEEDER_CPM_PCM, Constants.FEEDER_CPM_PISTON_FORWARD, Constants.FEEDER_CPM_PISTON_REVERSE);

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
  private final int ticksPerMotorRotation = Constants.CPM_NUM_ENCODER_TICS_PER_MOTOR_ROTATION;

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final I2C.Port m_i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(m_i2cPort);
  private AHRS m_navX = new AHRS(SPI.Port.kMXP); 
  //private DigitalInput m_contactSensor = new DigitalInput(Constants.CPM_DIGITAL_INPUT_CHANNEL);

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
    
  public ControlPanelManipulator() {

    m_config = new TalonSRXConfiguration();
    m_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

    // Creates a new Feeder
    // Common settings
    m_feederSpinner.configFactoryDefault();
    m_feederSpinner.configAllSettings(m_config);
    // TODO: Experiment with deadband as time allows. 
    //m_feederSpinner.configNeutralDeadband(0.003, Constants.CPM_SENSOR_TIMEOUTMS);
    m_feederSpinner.enableVoltageCompensation(true);
    m_feederSpinner.setInverted(false);
    m_feederSpinner.setSensorPhase(false);
    m_feederSpinner.setNeutralMode(NeutralMode.Brake);
    
    // CPM 
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    /* Motion Magic */
    m_feederSpinner.selectProfileSlot(0, 0);
    MAXIMUM_WHEEL_ACCELERATION = new DoublePreferenceConstant("CPM Acceleration", INIT_MAX_WHEEL_ACCELERATION);
    MAXIMUM_WHEEL_ACCELERATION.addChangeHandler(
      (Double acceleration) -> m_feederSpinner.configMotionAcceleration(convertWheelVelocityToMotorVelocity(acceleration)));
    spinner_kP = new DoublePreferenceConstant("CPM Spinner kP", 0);
    spinner_kP.addChangeHandler((Double kP) -> m_feederSpinner.config_kP(0, kP));
    spinner_kI = new DoublePreferenceConstant("CPM Spinner kI", 0);
    spinner_kI.addChangeHandler((Double kI) -> m_feederSpinner.config_kI(0, kI));
    spinner_kD = new DoublePreferenceConstant("CPM Spinner kD", 0);
    spinner_kD.addChangeHandler((Double kD) -> m_feederSpinner.config_kD(0, kD));
    spinner_kF = new DoublePreferenceConstant("CPM Spinner kF", 0);
    spinner_kF.addChangeHandler((Double kF) -> m_feederSpinner.config_kF(0, kF));

    m_feederSpinner.configMotionCruiseVelocity(convertWheelVelocityToMotorVelocity(MAXIMUM_WHEEL_VELOCITY));
    m_feederSpinner.configMotionAcceleration(convertWheelVelocityToMotorVelocity(MAXIMUM_WHEEL_ACCELERATION.getValue()));
  
    SmartDashboard.putBoolean("Zero CPM", false);
  }

  // Feeder
  public void setFeeder(double percentOutput) {
    m_feederSpinner.set(ControlMode.PercentOutput, percentOutput);
  }

  // CPM
  public void retractCPM() {
    m_deployer.set(Value.kReverse);
  }

  public void deployCPM() {
    m_deployer.set(Value.kForward);
  }
  
  public boolean isCPMEngaged() {
    //return m_contactSensor.get();
    return true; // right now there is no contact sensor for the CPM
  }

  public String getColor() {
    // Feeder
    retractCPM();

    // CPM
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

  public double calcPositionControlTargetPosition() {
    //calculate number of slices to move
    final String colorListString = "yrgb"; //string for what half the wheel looks like
    int robotSensorColorPosition = colorListString.indexOf(getColor()); // find position in string
    int gameSensorColorPosition = colorListString.indexOf(getFMSColorTarget()); // find position in string
    
    System.out.println("CPM: Robot Sensor Color Position: "+ robotSensorColorPosition);
    System.out.println("CPM: Game Sensor Color Position: "+ gameSensorColorPosition);
  
    int positionDistance = robotSensorColorPosition - gameSensorColorPosition;
    System.out.println("CPM: Distance in color array position"+positionDistance);
    double slicesToMove = 0.; // needs to account for the two slice offset between robot and game sensor
    int movementDirection = 1;
    
    if (positionDistance<0){ // reduces the number of cases in the switch statement
      movementDirection = -1;
    } 
    
    switch(Math.abs(positionDistance)) {
      // identify how many slices to move but also account for the 2 slice offset between sensors
      case 0:
        slicesToMove = 2.; 
        break;
      case 1:
        slicesToMove = 1. * movementDirection;
        break;
      case 2:
        slicesToMove = 0.;
        break;
      case 3:
        slicesToMove = -1. * movementDirection;
    }
    System.out.println("CPM: Slices to move: "+slicesToMove);

    return slicesToMove/Constants.CPM_NUM_SLICES;
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
    return m_feederSpinner.getSelectedSensorPosition();
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
    return convertMotorPositionToWheelPosition(m_feederSpinner.getSelectedSensorPosition());
  }

  public double getWheelVelocity() {
    return convertMotorVelocityToWheelVelocity(m_feederSpinner.getSelectedSensorVelocity());
  }
  
  public double getMotorVelocity() {
    return m_feederSpinner.getSelectedSensorVelocity();
  }

  public void setWheelPosition(double position) {
    m_feederSpinner.setSelectedSensorPosition(convertWheelPositionToMotorPosition(position));
  }

  public void setMotorSensorPosition(int sensorPosition){
    m_feederSpinner.setSelectedSensorPosition(sensorPosition);
  }

  public void moveWheelToPosition(double wheelPosition) {
    m_feederSpinner.set(ControlMode.MotionMagic, convertWheelPositionToMotorPosition(wheelPosition));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CPM Wheel Velocity", getWheelVelocity());
    SmartDashboard.putNumber("CPM Wheel Position", getWheelPosition());
    SmartDashboard.putNumber("CPM Motor Position", getMotorSensorPosition());
    SmartDashboard.putNumber("CPM Motor Velocity", getMotorVelocity());

    SmartDashboard.putNumber("CPM Target Position", convertMotorPositionToWheelPosition(m_feederSpinner.getActiveTrajectoryPosition()));
    SmartDashboard.putNumber("CPM Target Velocity", convertMotorVelocityToWheelVelocity(m_feederSpinner.getActiveTrajectoryVelocity()));

    SmartDashboard.putString("CPM sensor color", getColor());
    SmartDashboard.putNumber("Red", getRawColor().red);
    SmartDashboard.putNumber("Green", getRawColor().green);
    SmartDashboard.putNumber("Blue", getRawColor().blue);

    if(SmartDashboard.getBoolean("Zero CPM Sensor", false)) {
      setMotorSensorPosition(0);
      System.out.println("CPM: Setting wheel sensor postion to 0");
    }

    m_feederSpinner.config_kP(0, spinner_kP.getValue());
    m_feederSpinner.config_kI(0, spinner_kI.getValue());
    m_feederSpinner.config_kD(0, spinner_kD.getValue());
    m_feederSpinner.config_kF(0, spinner_kF.getValue());
  }
}
