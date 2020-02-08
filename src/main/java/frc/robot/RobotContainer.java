/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.climber.ExtendLeftClimber;
import frc.robot.commands.climber.ExtendRightClimber;
import frc.robot.commands.MoveColorWheelToTargetColor;
import frc.robot.commands.ReportColor;
import frc.robot.commands.climber.RetractClimber;
import frc.robot.commands.climber.StopClimber;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RotateColorWheel;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.climber.TiltClimberLeft;
import frc.robot.commands.climber.TiltClimberRight;
import frc.robot.subsystems.Climber;
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

  private final Climber m_climber = new Climber();

  // Intake Commands
  private final DeployIntake m_deployIntake = new DeployIntake(m_intake);
  private final RetractIntake m_retractIntake = new RetractIntake(m_intake);
  private final StopIntake m_stopIntake = new StopIntake(m_intake);
  private final RunIntake m_runIntake = new RunIntake(m_intake, 1);
  private final RunIntake m_ejectIntake = new RunIntake(m_intake, -1);

  // CPM Commands
  private final ReportColor m_reportColor= new ReportColor(m_cpm);
  private final MoveColorWheelToTargetColor m_moveColorWheelToTargetColor = new MoveColorWheelToTargetColor(m_cpm);
  private final RotateColorWheel m_rotateColorWheel = new RotateColorWheel(m_cpm);
  private final TJController m_driverController = new TJController(0);
  private final TJController m_operatorController = new TJController(1);

  // Climber Commands
  private final ExtendClimber m_extendClimber;
  private final RetractClimber m_retractClimber;
  private final TiltClimberLeft m_tiltClimberLeft;
  private final TiltClimberRight m_tiltClimberRight;
  private final ExtendRightClimber m_extendRightClimber;
  private final ExtendLeftClimber m_extendLeftClimber;


  private final StopClimber m_stopClimber = new StopClimber(m_climber);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    DoubleSupplier climbSpeedSingleSupplier = m_driverController::getRightStickX;
    DoubleSupplier climbSpeedSupplier = m_driverController::getRightStickY;
    
    m_extendClimber = new ExtendClimber(m_climber, climbSpeedSupplier);
    m_retractClimber = new RetractClimber(m_climber, climbSpeedSupplier);
    m_tiltClimberLeft = new TiltClimberLeft(m_climber, climbSpeedSingleSupplier);
    m_tiltClimberRight = new TiltClimberRight(m_climber, climbSpeedSingleSupplier);
    m_extendLeftClimber = new ExtendLeftClimber(m_climber, climbSpeedSingleSupplier);
    m_extendRightClimber = new ExtendRightClimber(m_climber, climbSpeedSingleSupplier);

    

    // Configure the button bindings
    configureButtonBindings();

    m_intake.setDefaultCommand(m_stopIntake);

    m_climber.setDefaultCommand(m_stopClimber);


    /**
     * Climber Commands
     * 
     * If the left trigger is held, the right stick will move both climbers.
     * If it is not held, the right climber is controlled by the right stick y-axis, and
     * the left climber is controlled by the right stick x-axis.
     * 
     * Can be implemented onto the button box; trigger replaced with button held, and 
     * controller stick replaced by operator console joystick.
     */

    if(m_driverController.getLeftTrigger() != 0) {
      if(m_driverController.getRightStickY() > Constants.CONTROLLER_DEADZONE) {
        m_extendClimber.execute();
      } else {
        m_retractClimber.execute();
      }
    } else {
      if(m_driverController.getRightStickY() > Constants.CONTROLLER_DEADZONE) {
        m_extendRightClimber.execute();
      } else if (m_driverController.getRightStickY() < -1 * Constants.CONTROLLER_DEADZONE) {
        m_extendLeftClimber.execute();
      } else if (m_driverController.getRightStickX() > Constants.CONTROLLER_DEADZONE) {
        m_tiltClimberRight.execute();
      } else if (m_driverController.getRightStickX() < -1 * Constants.CONTROLLER_DEADZONE) {
        m_tiltClimberLeft.execute();
      }
    }

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver Controller
    m_driverController.buttonB.whileHeld(m_reportColor);
    m_driverController.buttonA.whenPressed(m_moveColorWheelToTargetColor);
    m_driverController.buttonY.whenPressed(m_rotateColorWheel);
    // Climber test on driver controller
    


    // Operator Controller
    // m_operatorController.buttonY.whileHeld(m_retractClimber); //up
    // // m_operatorController.buttonX.whileHeld(m_extendClimber); //down
    // // m_operatorController.buttonA.whileHeld(m_tiltClimberLeft); //left
    // // m_operatorController.buttonB.whileHeld(m_tiltClimberRight); //right
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
