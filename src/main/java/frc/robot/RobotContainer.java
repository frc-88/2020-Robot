/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.cpm.*;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.vision.SetToFrontCamera;
import frc.robot.commands.vision.SetToRearCamera;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CalculateDriveEfficiency;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.drive.TestDriveStaticFriction;
import frc.robot.commands.drive.TurnToHeading;
import frc.robot.driveutil.DriveUtils;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.util.TJController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private final Sensors m_sensors = new Sensors();

  private final TJController m_driverController = new TJController(1);
  private final TJController m_operatorController = new TJController(0);

  // Subsystems
  private final ControlPanelManipulator m_cpm = new ControlPanelManipulator();


  // Commands
  private final Command m_autoCommand = new WaitCommand(1);
  private final ReportColor m_reportColor = new ReportColor(m_cpm);

  // private final MoveColorWheelToTargetColor m_moveColorWheelToTargetColor = new MoveColorWheelToTargetColor(m_cpm);
  // private final RotateColorWheel m_rotateColorWheel = new RotateColorWheel(m_cpm);

    
  // CPM Commands
  private final SetColorWheelPosition m_setColorWheelPosition = new SetColorWheelPosition(m_cpm, m_operatorController);
  private final RotateColorWheel m_rotateColorWheel = new RotateColorWheel(m_cpm, m_operatorController);
  private final MoveColorWheelToTargetColor m_moveColorWheelToTargetColor = new MoveColorWheelToTargetColor(m_cpm, m_operatorController);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    SmartDashboard.putData(m_setColorWheelPosition);
    SmartDashboard.putData(m_rotateColorWheel);
    SmartDashboard.putData(m_moveColorWheelToTargetColor);

    m_operatorController.buttonA.whenPressed(m_setColorWheelPosition);
    m_operatorController.buttonB.whenPressed(m_rotateColorWheel);
    m_operatorController.buttonY.whenPressed(m_moveColorWheelToTargetColor);

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
