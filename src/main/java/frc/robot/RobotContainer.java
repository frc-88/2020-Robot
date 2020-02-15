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
import frc.robot.commands.drive.*;
import frc.robot.commands.hopper.*;
import frc.robot.driveutil.DriveUtils;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;
import frc.robot.util.TJController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
  // Subsystems
  private final Sensors m_sensors = new Sensors();
  private final Drive m_drive = new Drive(m_sensors);
  private final Hopper m_hopper = new Hopper();
  // private final ControlPanelManipulator m_cpm = new ControlPanelManipulator();
  // private final Intake m_intake = new Intake();

  // Controllers
  private final TJController m_driverController = new TJController(0);

  // Commands
  private final Command m_autoCommand = new WaitCommand(1);

  // private final DeployIntake m_deployIntake = new DeployIntake(m_intake);
  // private final RetractIntake m_retractIntake = new RetractIntake(m_intake);
  // private final StopIntake m_stopIntake = new StopIntake(m_intake);
  // private final RunIntake m_runIntake = new RunIntake(m_intake, 1);
  // private final RunIntake m_ejectIntake = new RunIntake(m_intake, -1);

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Initialize everything
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

    // Configure the button bindings
    configureButtonBindings();
    configureSmartDashboardButtons();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    // m_driverController.buttonB.whileHeld(m_reportColor);
    // m_driverController.buttonA.whenPressed(m_moveColorWheelToTargetColor);
    // m_driverController.buttonY.whenPressed(m_rotateColorWheel);

    // m_driverController.buttonRightBumper.whenPressed(m_setToFrontCamera);
    // m_driverController.buttonRightBumper.whenReleased(m_setToRearCamera);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("TestDriveStaticFriction", m_testDriveStaticFriction);
    SmartDashboard.putData("CalculateDriveEfficiency", m_calculateDriveEfficiency);

    SmartDashboard.putNumber("SetTestDriveSpeed", 0);
    SmartDashboard.putNumber("SetTestDriveTurn", 0);
    SmartDashboard.putBoolean("SetTestShiftHigh", false);
    SmartDashboard.putData("TestArcadeDrive", m_testArcadeDrive);

    SmartDashboard.putNumber("SetTestHeading", 0);
    SmartDashboard.putData("TestTurnToHeading", new InstantCommand(() -> (new TurnToHeading(m_drive, m_sensors, SmartDashboard.getNumber("SetTestHeading", 0))).schedule()));

    SmartDashboard.putData("Hopper Intake", new HopperIntakeMode(m_hopper));
    SmartDashboard.putData("Hopper Shoot", new HopperShootMode(m_hopper));
    SmartDashboard.putData("Hopper Stop", new HopperStop(m_hopper));
  }

  private void configureDefaultCommands() {
    // m_intake.setDefaultCommand(m_stopIntake);
    m_drive.setDefaultCommand(m_arcadeDrive);
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
