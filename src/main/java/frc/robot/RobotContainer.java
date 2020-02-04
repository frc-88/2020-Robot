/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.MoveColorWheelToTargetColor;
import frc.robot.commands.ReportColor;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RotateColorWheel;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CalculateDriveEfficiency;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.drive.TestDriveStaticFriction;
import frc.robot.driveutil.DriveUtils;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.util.TJController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Controllers
  private final TJController m_driverController = new TJController(0);

  // Subsystems
  private final ControlPanelManipulator m_cpm = new ControlPanelManipulator();
  private final Intake m_intake = new Intake();
  private final Drive m_drive = new Drive();

  // Commands
  private final Command m_autoCommand = new WaitCommand(1);

  private final DeployIntake m_deployIntake = new DeployIntake(m_intake);
  private final RetractIntake m_retractIntake = new RetractIntake(m_intake);
  private final StopIntake m_stopIntake = new StopIntake(m_intake);
  private final RunIntake m_runIntake = new RunIntake(m_intake, 1);
  private final RunIntake m_ejectIntake = new RunIntake(m_intake, -1);

  private final ReportColor m_reportColor = new ReportColor(m_cpm);

  private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drive,
      DriveUtils.deadbandExponential(m_driverController::getLeftStickY, Constants.DRIVE_SPEED_EXP, Constants.DRIVE_JOYSTICK_DEADBAND),
      DriveUtils.invertDouble(DriveUtils.deadbandExponential(m_driverController::getRightStickX, Constants.DRIVE_SPEED_EXP, Constants.DRIVE_JOYSTICK_DEADBAND)),
      m_drive::autoshift);
  private final TankDrive m_tankDrive = new TankDrive(m_drive, m_driverController::getLeftStickY,
      m_driverController::getRightStickY);

  private final TestDriveStaticFriction m_testDriveStaticFriction = new TestDriveStaticFriction(m_drive);
  private final CalculateDriveEfficiency m_calculateDriveEfficiency = new CalculateDriveEfficiency(m_drive);
  private final ArcadeDrive m_testArcadeDrive = new ArcadeDrive(m_drive, 
      () -> SmartDashboard.getNumber("SetTestDriveSpeed", 0) / (SmartDashboard.getBoolean("SetTestShiftHigh", false) ? Constants.MAX_SPEED_HIGH : Constants.MAX_SPEED_LOW), 
      () -> SmartDashboard.getNumber("SetTestDriveTurn", 0) / (SmartDashboard.getBoolean("SetTestShiftHigh", false) ? Constants.MAX_SPEED_HIGH : Constants.MAX_SPEED_LOW),
      () -> SmartDashboard.getBoolean("SetTestShiftHigh", false));

  private final MoveColorWheelToTargetColor m_moveColorWheelToTargetColor = new MoveColorWheelToTargetColor(m_cpm);
  private final RotateColorWheel m_rotateColorWheel = new RotateColorWheel(m_cpm);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_intake.setDefaultCommand(m_stopIntake);
    // m_drive.setDefaultCommand(m_arcadeDrive);
    m_drive.setDefaultCommand(m_arcadeDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.buttonB.whileHeld(m_reportColor);
    m_driverController.buttonA.whenPressed(m_moveColorWheelToTargetColor);
    m_driverController.buttonY.whenPressed(m_rotateColorWheel);

    SmartDashboard.putData("TestDriveStaticFriction", m_testDriveStaticFriction);
    SmartDashboard.putData("CalculateDriveEfficiency", m_calculateDriveEfficiency);

    SmartDashboard.putNumber("SetTestDriveSpeed", 0); 
    SmartDashboard.putNumber("SetTestDriveTurn", 0);
    SmartDashboard.putBoolean("SetTestShiftHigh", false);
    SmartDashboard.putData("TestArcadeDrive", m_testArcadeDrive);
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
