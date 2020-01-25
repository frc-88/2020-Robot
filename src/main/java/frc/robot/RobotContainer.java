/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.ReportColor;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.subsystems.Intake;
import frc.robot.util.TJController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Command m_autoCommand = new WaitCommand(1);

  private final ControlPanelManipulator m_cpm= new ControlPanelManipulator();

  private final Intake m_intake = new Intake();

  private final DeployIntake m_deployIntake = new DeployIntake(m_intake);
  private final RetractIntake m_retractIntake = new RetractIntake(m_intake);
  private final StopIntake m_stopIntake = new StopIntake(m_intake);
  private final RunIntake m_runIntake = new RunIntake(m_intake, 1);
  private final RunIntake m_ejectIntake = new RunIntake(m_intake, -1);


  private final ReportColor m_reportColor= new ReportColor(m_cpm);

  private final TJController m_driverController = new TJController(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_intake.setDefaultCommand(m_stopIntake);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.buttonB.whileHeld(m_reportColor);
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
