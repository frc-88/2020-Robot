/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.hopper.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.climber.DisengageRatchets;
import frc.robot.commands.climber.EngageRatchets;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.shooter.*;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CalculateDriveEfficiency;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.drive.TestDriveStaticFriction;
import frc.robot.commands.drive.TurnToHeading;
import frc.robot.driveutil.DriveUtils;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ButtonBox;
import frc.robot.util.TJController;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Sensors m_sensors = new Sensors();

  private final TJController m_driverController = new TJController(0);
  private final ButtonBox m_buttonBox = new ButtonBox(1);
  private final TJController m_testController = new TJController(2);

  // Subsystems
  private final Drive m_drive = new Drive(m_sensors);
  private final Climber m_climber = new Climber();
  private final Hopper m_hopper = new Hopper();
  private final Arm m_arm = new Arm(m_driverController::isButtonStartPressed);
  private final Shooter m_shooter = new Shooter();
  // private final ControlPanelManipulator m_cpm = new ControlPanelManipulator();
  private final Intake m_intake = new Intake();

  // Commands
  private final Command m_autoCommand = new WaitCommand(1);

  // Climber Commands

  private final RunClimber m_runClimber;
  private final EngageRatchets m_engageRatchets = new EngageRatchets(m_climber);
  private final DisengageRatchets m_disengageRatchets = new DisengageRatchets(m_climber);

  // private final DeployIntake m_deployIntake = new DeployIntake(m_intake);
  // private final RetractIntake m_retractIntake = new RetractIntake(m_intake);
  // private final StopIntake m_stopIntake = new StopIntake(m_intake);
  // private final RunIntake m_runIntake = new RunIntake(m_intake, 1);
  // private final RunIntake m_ejectIntake = new RunIntake(m_intake, -1);
  private final DeployIntake m_deployIntake = new DeployIntake(m_intake);
  private final RetractIntake m_retractIntake = new RetractIntake(m_intake);
  private final StopIntake m_stopIntake = new StopIntake(m_intake);
  private final RunIntake m_runIntake = new RunIntake(m_intake, 1);
  private final RunIntake m_ejectIntake = new RunIntake(m_intake, -1);

  // private final ReportColor m_reportColor = new ReportColor(m_cpm);

  private final ArcadeDrive m_arcadeDrive;
  private final TankDrive m_tankDrive;

  private final TestDriveStaticFriction m_testDriveStaticFriction;
  private final CalculateDriveEfficiency m_calculateDriveEfficiency;
  private final ArcadeDrive m_testArcadeDrive;

  // private final MoveColorWheelToTargetColor m_moveColorWheelToTargetColor = new MoveColorWheelToTargetColor(m_cpm);
  // private final RotateColorWheel m_rotateColorWheel = new RotateColorWheel(m_cpm);

  // private final SetToFrontCamera m_setToFrontCamera = new SetToFrontCamera(m_sensors);
  // private final SetToRearCamera m_setToRearCamera = new SetToRearCamera(m_sensors);

  private final ArmMotionMagic m_stowArm;
  private final ArmMotionMagic m_armToLayup;
  private final CommandBase m_armToSmartDashboard;
  private final CommandBase m_armHoldCurrentPosition;
  private final RotateArm m_rotateArm;
  private final CalibrateArm m_calibrateArm;

  private final DoublePreferenceConstant m_armStowAngle = new DoublePreferenceConstant("Arm Stow Angle", 0);
  private final DoublePreferenceConstant m_armLayupAngle = new DoublePreferenceConstant("Arm Layup Angle", 45);

  private TestBrakeMode m_testBrakeMode = new TestBrakeMode(m_arm);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Initialize everything

    DoubleSupplier climbSpeedXSupplier = m_buttonBox::getClimberTiltAxis;
    DoubleSupplier climbSpeedYSupplier = m_buttonBox::getClimberSpeedAxis;    

    m_runClimber = new RunClimber(m_climber, climbSpeedXSupplier, climbSpeedYSupplier);

    DoubleSupplier arcadeDriveSpeedSupplier = DriveUtils.deadbandExponential(m_driverController::getLeftStickY,
        Constants.DRIVE_SPEED_EXP, Constants.DRIVE_JOYSTICK_DEADBAND);
    DoubleSupplier arcadeDriveTurnSupplier = DriveUtils.cheesyTurn(
        arcadeDriveSpeedSupplier, DriveUtils.deadbandExponential(m_driverController::getRightStickX,
            Constants.DRIVE_SPEED_EXP, Constants.DRIVE_JOYSTICK_DEADBAND),
        Constants.CHEESY_DRIVE_MIN_TURN, Constants.CHEESY_DRIVE_MAX_TURN);
    BooleanSupplier arcadeDriveShiftSupplier = () -> m_drive.autoshift(arcadeDriveSpeedSupplier.getAsDouble());
    m_arcadeDrive = new ArcadeDrive(m_drive, arcadeDriveSpeedSupplier, arcadeDriveTurnSupplier,
        arcadeDriveShiftSupplier);

    DoubleSupplier tankDriveLeftSupplier = m_driverController::getLeftStickY;
    DoubleSupplier tankDriveRightSupplier = m_driverController::getRightStickY;
    m_tankDrive = new TankDrive(m_drive, tankDriveLeftSupplier, tankDriveRightSupplier);

    m_testDriveStaticFriction = new TestDriveStaticFriction(m_drive);
    m_calculateDriveEfficiency = new CalculateDriveEfficiency(m_drive);

    BooleanSupplier testArcadeDriveShiftSupplier = () -> SmartDashboard.getBoolean("SetTestShiftHigh", false);
    DoubleSupplier testArcadeDriveSpeedSupplier = () -> SmartDashboard.getNumber("SetTestDriveSpeed", 0)
        / Constants.MAX_SPEED_HIGH;
    DoubleSupplier testArcadeDriveTurnSupplier = () -> SmartDashboard.getNumber("SetTestDriveTurn", 0)
        * (Constants.WHEEL_BASE_WIDTH * Math.PI) / Constants.MAX_SPEED_HIGH / 360;
    m_testArcadeDrive = new ArcadeDrive(m_drive, testArcadeDriveSpeedSupplier, testArcadeDriveTurnSupplier,
        testArcadeDriveShiftSupplier);

    m_stowArm = new ArmMotionMagic(m_arm, m_armStowAngle.getValue());
    m_armToLayup = new ArmMotionMagic(m_arm, m_armLayupAngle.getValue());

    m_armToSmartDashboard = new InstantCommand(() -> new ArmMotionMagic(m_arm, SmartDashboard.getNumber("ArmDashboardPosition", 0)).schedule());
    m_armHoldCurrentPosition = new InstantCommand(() -> new ArmMotionMagic(m_arm, m_arm.getCurrentArmPosition()).schedule(), m_arm);

    DoubleSupplier armSpeedSupplier = DriveUtils.deadbandExponential(m_testController::getRightStickY, Constants.ARM_SPEED_EXP, Constants.TEST_JOYSTICK_DEADBAND);
    m_rotateArm = new RotateArm(m_arm, armSpeedSupplier);

    m_calibrateArm = new CalibrateArm(m_arm);

    // Configure the button bindings
    configureButtonBindings();

    m_climber.setDefaultCommand(m_runClimber);
    // m_intake.setDefaultCommand(m_stopIntake);
    m_drive.setDefaultCommand(m_arcadeDrive);
    configureSmartDashboardButtons();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {

    m_buttonBox.button4.whenPressed(new SequentialCommandGroup(new DeployIntake(m_intake), new RunIntake(m_intake, 1.)));
    m_buttonBox.button4.whenReleased(new SequentialCommandGroup(new RetractIntake(m_intake), new StopIntake(m_intake)));

    m_driverController.buttonStart.whenHeld(m_testBrakeMode);

    // m_driverController.buttonB.whileHeld(m_reportColor);
    // m_driverController.buttonA.whenPressed(m_moveColorWheelToTargetColor);
    // m_driverController.buttonY.whenPressed(m_rotateColorWheel);

    // m_driverController.buttonRightBumper.whenPressed(m_setToFrontCamera);
    // m_driverController.buttonRightBumper.whenReleased(m_setToRearCamera);

    m_buttonBox.button8.whenPressed(m_engageRatchets);
    m_buttonBox.button8.whenReleased(m_disengageRatchets);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("TestDriveStaticFriction", m_testDriveStaticFriction);
    SmartDashboard.putData("CalculateDriveEfficiency", m_calculateDriveEfficiency);

    SmartDashboard.putNumber("SetTestDriveSpeed", 0);
    SmartDashboard.putNumber("SetTestDriveTurn", 0);
    SmartDashboard.putBoolean("SetTestShiftHigh", false);
    SmartDashboard.putData("TestArcadeDrive", m_testArcadeDrive);

    SmartDashboard.putData("Engage ratchets", m_engageRatchets);
    SmartDashboard.putData("Disengage ratchets", m_disengageRatchets);

    SmartDashboard.putNumber("SetTestHeading", 0);
    SmartDashboard.putData("TestTurnToHeading", new InstantCommand(() -> (new TurnToHeading(m_drive, m_sensors, SmartDashboard.getNumber("SetTestHeading", 0))).schedule()));

    SmartDashboard.putData("Deploy Intake", m_deployIntake);
    SmartDashboard.putData("Retract Intake", m_retractIntake);
    SmartDashboard.putData("Run Intake", m_runIntake);
    SmartDashboard.putData("Stop Intake", m_stopIntake);
    SmartDashboard.putData("Eject Intake", m_ejectIntake);

    SmartDashboard.putData("Hopper Intake", new HopperIntakeMode(m_hopper));
    SmartDashboard.putData("Hopper Shoot", new HopperShootMode(m_hopper));
    SmartDashboard.putData("Hopper Stop", new HopperStop(m_hopper));

    SmartDashboard.putData("Arm Manual", m_rotateArm);

    SmartDashboard.putNumber("ArmDashboardPosition", 0);
    SmartDashboard.putData("Arm to Position", m_armToSmartDashboard);

    SmartDashboard.putData("Calibrate Arm", m_calibrateArm);

    SmartDashboard.putNumber("ShooterTestFeederSpeed", 0);
    SmartDashboard.putNumber("ShooterTestFlywheelSpeed", 0);
    SmartDashboard.putData("ShooterTestFeeder", new InstantCommand(() -> (new ShooterFeederRun(m_shooter, SmartDashboard.getNumber("ShooterTestFeederSpeed", 0))).schedule()));
    SmartDashboard.putData("ShooterTestFlywheel", new InstantCommand(() -> (new ShooterFlywheelRun(m_shooter, SmartDashboard.getNumber("ShooterTestFlywheelSpeed", 0))).schedule()));
    SmartDashboard.putData("ShooterStopFeeder", new ShooterFeederRun(m_shooter, 0));
    SmartDashboard.putData("ShooterStopFlywheel", new ShooterFlywheelRun(m_shooter, 0));
    SmartDashboard.putData("ShooterStopAll", new ShooterStop(m_shooter));
    SmartDashboard.putData("ShooterFlywheelBasic", new ShooterFlywheelRunBasic(m_shooter, DriveUtils.deadbandExponential(m_testController::getLeftStickY, Constants.SHOOTER_FLYWHEEL_SPEED_EXP, Constants.TEST_JOYSTICK_DEADBAND)));
  }

  private void configureDefaultCommands() {
    // m_intake.setDefaultCommand(m_stopIntake);
    m_drive.setDefaultCommand(m_arcadeDrive);
    // m_arm.setDefaultCommand(m_armHoldCurrentPosition);
    m_intake.setDefaultCommand(m_stopIntake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
