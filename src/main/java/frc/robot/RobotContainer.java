/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.WaitForDriveAimed;
import frc.robot.commands.WaitForShooterReady;
import frc.robot.commands.WaitInitializeCommand;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.arm.ArmFullUp;
import frc.robot.commands.arm.ArmMotionMagic;
import frc.robot.commands.arm.ArmStow;
import frc.robot.commands.arm.CalibrateArm;
import frc.robot.commands.arm.RotateArm;
import frc.robot.commands.arm.TestBrakeMode;
import frc.robot.commands.climber.ClimberMaxRobotHeight;
import frc.robot.commands.climber.DisengageRatchets;
import frc.robot.commands.climber.EngageRatchets;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.climber.StopClimber;
import frc.robot.commands.climber.ZeroClimber;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.BasicAutoDrive;
import frc.robot.commands.drive.CalculateDriveEfficiency;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.drive.TestDriveStaticFriction;
import frc.robot.commands.drive.TurnToHeading;
import frc.robot.commands.drive.TurnToLimelight;
import frc.robot.commands.feeder.FeederRun;
import frc.robot.commands.feeder.FeederStop;
import frc.robot.commands.hopper.HopperEject;
import frc.robot.commands.hopper.HopperIntakeFinishMode;
import frc.robot.commands.hopper.HopperIntakeMode;
import frc.robot.commands.hopper.HopperShootMode;
import frc.robot.commands.hopper.HopperShootUnjamMode;
import frc.robot.commands.hopper.HopperStop;
import frc.robot.commands.hopper.HopperTest;
import frc.robot.commands.shooter.ShooterFlywheelRun;
import frc.robot.commands.shooter.ShooterFlywheelRunBasic;
import frc.robot.commands.shooter.ShooterRunFromLimelight;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.sensors.LimelightToggle;
import frc.robot.commands.sensors.WaitForBallsShot;
import frc.robot.driveutil.DriveUtils;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ButtonBox;
import frc.robot.util.TJController;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class RobotContainer {

  /***
  *    .______   .______       _______  _______  _______ .______       _______ .__   __.   ______  _______     _______.
  *    |   _  \  |   _  \     |   ____||   ____||   ____||   _  \     |   ____||  \ |  |  /      ||   ____|   /       |
  *    |  |_)  | |  |_)  |    |  |__   |  |__   |  |__   |  |_)  |    |  |__   |   \|  | |  ,----'|  |__     |   (----`
  *    |   ___/  |      /     |   __|  |   __|  |   __|  |      /     |   __|  |  . `  | |  |     |   __|     \   \    
  *    |  |      |  |\  \----.|  |____ |  |     |  |____ |  |\  \----.|  |____ |  |\   | |  `----.|  |____.----)   |   
  *    | _|      | _| `._____||_______||__|     |_______|| _| `._____||_______||__| \__|  \______||_______|_______/    
  *                                                                                                                    
  */

  private final DoublePreferenceConstant m_armLayupAngle = new DoublePreferenceConstant("Arm Layup Angle", 45);
  private final DoublePreferenceConstant m_shooterLayupSpeed = new DoublePreferenceConstant("Shooter Layup Speed", 2500);
  private final DoublePreferenceConstant m_shooterUpSpeed = new DoublePreferenceConstant("Shooter Up Speed", 5100);


  /***
  *      ______   ______   .__   __. .___________..______        ______    __       __       _______ .______          _______.
  *     /      | /  __  \  |  \ |  | |           ||   _  \      /  __  \  |  |     |  |     |   ____||   _  \        /       |
  *    |  ,----'|  |  |  | |   \|  | `---|  |----`|  |_)  |    |  |  |  | |  |     |  |     |  |__   |  |_)  |      |   (----`
  *    |  |     |  |  |  | |  . `  |     |  |     |      /     |  |  |  | |  |     |  |     |   __|  |      /        \   \    
  *    |  `----.|  `--'  | |  |\   |     |  |     |  |\  \----.|  `--'  | |  `----.|  `----.|  |____ |  |\  \----.----)   |   
  *     \______| \______/  |__| \__|     |__|     | _| `._____| \______/  |_______||_______||_______|| _| `._____|_______/    
  *                                                                                                                           
  */

  private final TJController m_driverController = new TJController(0);
  private final ButtonBox m_buttonBox = new ButtonBox(1);
  private final TJController m_testController = new TJController(2);


  /***
  *         _______. __    __  .______        _______.____    ____  _______.___________. _______ .___  ___.      _______.
  *        /       ||  |  |  | |   _  \      /       |\   \  /   / /       |           ||   ____||   \/   |     /       |
  *       |   (----`|  |  |  | |  |_)  |    |   (----` \   \/   / |   (----`---|  |----`|  |__   |  \  /  |    |   (----`
  *        \   \    |  |  |  | |   _  <      \   \      \_    _/   \   \       |  |     |   __|  |  |\/|  |     \   \    
  *    .----)   |   |  `--'  | |  |_)  | .----)   |       |  | .----)   |      |  |     |  |____ |  |  |  | .----)   |   
  *    |_______/     \______/  |______/  |_______/        |__| |_______/       |__|     |_______||__|  |__| |_______/    
  *                                                                                                                      
  */

  private final Sensors m_sensors = new Sensors(m_driverController::isButtonAPressed);
  private final Drive m_drive = new Drive(m_sensors);
  private final Climber m_climber = new Climber();
  private final Arm m_arm = new Arm(m_driverController::isButtonStartPressed);
  private final Feeder m_feeder = new Feeder();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter(m_sensors);
  private final Intake m_intake = new Intake();
  // private final ControlPanelManipulator m_cpm = new ControlPanelManipulator();

  /***
  *      ______   ______   .___  ___. .___  ___.      ___      .__   __.  _______       _______.
  *     /      | /  __  \  |   \/   | |   \/   |     /   \     |  \ |  | |       \     /       |
  *    |  ,----'|  |  |  | |  \  /  | |  \  /  |    /  ^  \    |   \|  | |  .--.  |   |   (----`
  *    |  |     |  |  |  | |  |\/|  | |  |\/|  |   /  /_\  \   |  . `  | |  |  |  |    \   \    
  *    |  `----.|  `--'  | |  |  |  | |  |  |  |  /  _____  \  |  |\   | |  '--'  |.----)   |   
  *     \______| \______/  |__|  |__| |__|  |__| /__/     \__\ |__| \__| |_______/ |_______/    
  *                                                                                             
  */
  
  private CommandBase m_arcadeDrive;
  private CommandBase m_testArcadeDrive;

  // prepShoot - move the arm into shooting position and get the flywheel going
  //    turn on the limelight unless for layup
  private CommandBase m_prepShoot = new ConditionalCommand(
    new ParallelCommandGroup(
      new ArmFullUp(m_arm),
      new ShooterRunFromLimelight(m_shooter),
      new LimelightToggle(m_sensors, true)
    ),
    new ParallelCommandGroup(
      new ArmMotionMagic(m_arm, m_armLayupAngle.getValue()),
      new ShooterFlywheelRun(m_shooter, m_shooterLayupSpeed.getValue())
    ),
    m_buttonBox.button7::get);

  // shoot - runs shooter, moves arm to shooting angle, waits to be ready
  //    when ready, run feeder
  //    TODO - freeze driver control and aim using vision data
  private final CommandBase m_shoot =
    new ConditionalCommand(
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new ArmFullUp(m_arm), 
          new ShooterRunFromLimelight(m_shooter),
          new ParallelCommandGroup(
            new WaitForShooterReady(m_arm, m_shooter),
            new WaitForDriveAimed(m_drive)
          )
        ),
        new ParallelCommandGroup(
          new HopperShootMode(m_hopper), 
          new ArmFullUp(m_arm),
          new ShooterRunFromLimelight(m_shooter), 
          new FeederRun(m_feeder, 1.0),
          new RunIntake(m_intake, 0.3)
        )
      ),
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new ArmMotionMagic(m_arm, m_armLayupAngle.getValue()),
          new ShooterFlywheelRun(m_shooter, m_shooterLayupSpeed.getValue()),
          new WaitForShooterReady(m_arm, m_shooter)
        ),
        new ParallelCommandGroup(
          new HopperShootMode(m_hopper), 
          new ArmMotionMagic(m_arm, m_armLayupAngle.getValue()),
          new ShooterFlywheelRun(m_shooter, m_shooterLayupSpeed.getValue()), 
          new FeederRun(m_feeder, 1.0),
          new RunIntake(m_intake, 0.3)
        )
      ),
      m_buttonBox.button7::get);

  // pauseShoot - stop the hopper and the feeder maintain arm and flywheel for shooting
  private final CommandBase m_pauseShoot = 
    new ConditionalCommand(
      new SequentialCommandGroup(
          new ParallelRaceGroup(
            new HopperEject(m_hopper, -1.),
            new WaitCommand(1),
            new FeederStop(m_feeder), 
            new ArmFullUp(m_arm),
            new ShooterRunFromLimelight(m_shooter)),
        new ParallelCommandGroup(
          new HopperStop(m_hopper),
          new FeederStop(m_feeder), 
          new ArmFullUp(m_arm),
          new ShooterRunFromLimelight(m_shooter))),
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new HopperEject(m_hopper, -1.),
            new WaitCommand(1),
            new FeederStop(m_feeder),
            new ArmMotionMagic(m_arm, m_armLayupAngle.getValue()),
            new ShooterFlywheelRun(m_shooter, m_shooterLayupSpeed.getValue())),
          new ParallelCommandGroup(
            new HopperStop(m_hopper),
            new FeederStop(m_feeder),
            new ArmMotionMagic(m_arm, m_armLayupAngle.getValue()),
            new ShooterFlywheelRun(m_shooter, m_shooterLayupSpeed.getValue()))),
      m_buttonBox.button7::get);

  // stopShoot - stows arm, stops shooter, stops feeder, turns limelight off
  private final CommandBase m_stopShoot = 
    new ParallelCommandGroup(
      new LimelightToggle(m_sensors, false),
      new ArmStow(m_arm, () -> m_driverController.getRawButton(5)), 
      new ShooterStop(m_shooter), 
      new FeederStop(m_feeder)
    );

  // activateIntake - deploys and runs the intake, run hopper in intake mode
  private final CommandBase m_activateIntake = 
    new SequentialCommandGroup(
      new DeployIntake(m_intake),
      new RunIntake(m_intake, 1.)
    );

  // deactivateIntake - retracts the intake, stops the rollers and hopper after a delay
  private final CommandBase m_deactivateIntake = 
    new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitCommand(0.75),
        new RetractIntake(m_intake)
      ),
      new StopIntake(m_intake)
    );

  // regurgitate - deploy the intake and run the intake, hopper, and feeder in reverse
  private final CommandBase m_regurgitate = 
    new SequentialCommandGroup(
      new DeployIntake(m_intake),
      new ParallelCommandGroup(
        new RunIntake(m_intake, -1.), 
        new HopperEject(m_hopper, 1.),
        new FeederRun(m_feeder, -1.)
      )
    );

  // regurgitateStop - stop the hopper and feeder and retract the intake, stop intake rollers after a delay
  private final CommandBase m_regurgitateStop = 
    new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitCommand(0.75), 
        new HopperStop(m_hopper),
        new FeederStop(m_feeder), 
        new RetractIntake(m_intake)
      ),
      new StopIntake(m_intake)
    );

  // The currently running FASH (Feeder/Arm/Shooter/Hopper) combined command
  private CommandBase m_currentFASHCommand = m_stopShoot;

  //TODO: Make this look nice
  /*****
   * AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO
   * AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO
   * AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO
   * AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO AUTO
   */

  private class AutoClimber extends SequentialCommandGroup {
    public AutoClimber() {
      super (
        new ConditionalCommand(
          new EngageRatchets(m_climber),
          new DisengageRatchets(m_climber),
          m_buttonBox.button14::get),
        new ZeroClimber(m_climber),
        new StopClimber(m_climber)
      );
    }
  }

  private class AutoDoNothing extends ParallelCommandGroup {
    public AutoDoNothing() {
      super(
        new ArcadeDrive(m_drive, () -> 0, () -> 0, () -> false, () -> Constants.MAX_SPEED_HIGH),
        new ShooterRunFromLimelight(m_shooter),
        new FeederStop(m_feeder),
        new HopperStop(m_hopper),
        new StopIntake(m_intake),
        new ArmStow(m_arm, () -> false));
    }
  }

  private class AutoDelayedDrive extends SequentialCommandGroup {
    public AutoDelayedDrive() {
      super(
        new ParallelDeadlineGroup(
          new WaitCommand(SmartDashboard.getNumber("Auto Drive Wait", 10)),
          new ArcadeDrive(m_drive, () -> 0, () -> 0, () -> false, () -> Constants.MAX_SPEED_HIGH)
        ),
        new BasicAutoDrive(m_drive, SmartDashboard.getNumber("Auto Drive Distance", -2), SmartDashboard.getNumber("Auto Drive Distance", -2), 3),
        new ArcadeDrive(m_drive, () -> 0, () -> 0, () -> false, () -> Constants.MAX_SPEED_HIGH)
      );
    }
  }

  private class AutoShoot extends SequentialCommandGroup {
    public AutoShoot(int numBalls, double minShotTime, double maxShotTime, boolean aim) {
      super(
        new ParallelDeadlineGroup(
          new WaitCommand(.15),
          new LimelightToggle(m_sensors, true),
          new ArcadeDrive(m_drive, () -> 0, () -> 0, () -> false, () -> Constants.MAX_SPEED_HIGH),
          new ArmFullUp(m_arm), 
          new ShooterRunFromLimelight(m_shooter),
          new FeederStop(m_feeder),
          new RunIntake(m_intake, 0)
        ),
        new ParallelDeadlineGroup(
          new ParallelRaceGroup(
            new ParallelCommandGroup(
              new WaitForShooterReady(m_arm, m_shooter),
              new ConditionalCommand(new WaitForDriveAimed(m_drive), new WaitCommand(0.01), () -> aim),
              new WaitCommand(1.0)
            ),
            new WaitCommand(2.0)
          ),
          new ConditionalCommand(new TurnToLimelight(m_drive, m_sensors), new WaitCommand(0.01), () -> aim),
          new ArmFullUp(m_arm), 
          new ShooterRunFromLimelight(m_shooter),
          new FeederStop(m_feeder),
          new RunIntake(m_intake, 0)
        ),
        new ParallelDeadlineGroup(
          new ParallelRaceGroup(
            new ParallelCommandGroup(
              new WaitForBallsShot(m_sensors, numBalls),
              new WaitCommand(minShotTime)
            ),
            new WaitCommand(maxShotTime)
          ),
          new ConditionalCommand(new TurnToLimelight(m_drive, m_sensors), new WaitCommand(0.01), () -> aim),
          new ArmFullUp(m_arm), 
          new ShooterRunFromLimelight(m_shooter),
          new FeederRun(m_feeder, 1.0),
          new HopperShootMode(m_hopper),
          new SequentialCommandGroup(
            new ParallelDeadlineGroup(
              new WaitCommand(1.5),
              new RunIntake(m_intake, 0.3)
            ),
            new ParallelDeadlineGroup(
              new WaitCommand(.25),
              new DeployIntake(m_intake)
            ),
            new RetractIntake(m_intake),
            new RunIntake(m_intake, 0.3)
          )
        ),
        new LimelightToggle(m_sensors, false)
      );
    }
  }
  
  private final CommandBase m_autoDoNothing = new ParallelCommandGroup(
    new AutoDoNothing(),
    new AutoClimber()
  );

  private final CommandBase m_autoJustDrive = new ParallelCommandGroup(
    new SequentialCommandGroup(
      new ParallelDeadlineGroup(
          new WaitInitializeCommand(() -> SmartDashboard.getNumber("Auto Drive Wait", 10)),
          new ArcadeDrive(m_drive, () -> 0, () -> 0, () -> false, () -> Constants.MAX_SPEED_HIGH)
      ),
      new BasicAutoDrive(m_drive, () -> SmartDashboard.getNumber("Auto Drive Distance", -2), () -> SmartDashboard.getNumber("Auto Drive Distance", -2), 3),
      new ArcadeDrive(m_drive, () -> 0, () -> 0, () -> false, () -> Constants.MAX_SPEED_HIGH)
    ),
    new AutoClimber(),
    new ShooterStop(m_shooter),
    new FeederStop(m_feeder),
    new HopperStop(m_hopper),
    new StopIntake(m_intake),
    new ArmStow(m_arm, () -> m_driverController.getRawButton(5))
    );

  private final CommandBase m_auto3Ball = new ParallelCommandGroup(
    new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitInitializeCommand(() -> SmartDashboard.getNumber("Auto Drive Wait", 10)),
        new SequentialCommandGroup(
          new AutoShoot(100, 15, 15, false),
          new AutoDoNothing()
        )
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new BasicAutoDrive(m_drive, () -> SmartDashboard.getNumber("Auto Drive Distance", -2), () -> SmartDashboard.getNumber("Auto Drive Distance", -2), 3),
          new ArcadeDrive(m_drive, () -> 0, () -> 0, () -> false, () -> Constants.MAX_SPEED_HIGH)
        ), 
        new ShooterStop(m_shooter),
        new FeederStop(m_feeder),
        new HopperStop(m_hopper),
        new StopIntake(m_intake),
        new ArmStow(m_arm, () -> m_driverController.getRawButton(5)))
    ),
    new AutoClimber()
  );

  private final CommandBase m_autoTrench7Ball = new ParallelCommandGroup(
    new SequentialCommandGroup(
      new AutoShoot(3, .3, 3, false),
      new ParallelDeadlineGroup(
        new BasicAutoDrive(m_drive, -14, -15, 6),
        new ShooterRunFromLimelight(m_shooter),
        new FeederStop(m_feeder),
        new HopperStop(m_hopper),
        new SequentialCommandGroup(
          new DeployIntake(m_intake),
          new RunIntake(m_intake, 1)
        ),
        new ArmStow(m_arm, () -> m_driverController.getRawButton(5))     
        ),
      new ParallelDeadlineGroup(
        new BasicAutoDrive(m_drive, 9.7, 10.2, 6),
        new ShooterRunFromLimelight(m_shooter),
        new FeederStop(m_feeder),
        new HopperStop(m_hopper),
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(0.75),
            new RetractIntake(m_intake)
          ),
          new StopIntake(m_intake)
        ),
        new ArmStow(m_arm, () -> m_driverController.getRawButton(5))
      ),
      new AutoShoot(100, 15, 15, true)
    ),
    new AutoClimber()
  );

  private CommandBase m_autoCommand = m_autoTrench7Ball;
  private class ButtonAutoPair {
    private Trigger button;
    private CommandBase auto;

    public ButtonAutoPair(Trigger button, CommandBase auto) {
      this.button = button;
      this.auto = auto;
    }

    public void check() {
      if (this.button.get()) {
        m_autoCommand = this.auto;
      }
    }
  }
  private final List<ButtonAutoPair> autoSelectors = Arrays.asList(
    new ButtonAutoPair(m_buttonBox.button2, m_autoDoNothing),
    new ButtonAutoPair(m_buttonBox.button3, m_autoJustDrive),
    new ButtonAutoPair(m_buttonBox.button4, m_auto3Ball),
    new ButtonAutoPair(m_buttonBox.button5, m_autoTrench7Ball)
  );

  /***
  *      ______   ______   .__   __.      _______.___________..______       __    __    ______ .___________.  ______   .______      
  *     /      | /  __  \  |  \ |  |     /       |           ||   _  \     |  |  |  |  /      ||           | /  __  \  |   _  \     
  *    |  ,----'|  |  |  | |   \|  |    |   (----`---|  |----`|  |_)  |    |  |  |  | |  ,----'`---|  |----`|  |  |  | |  |_)  |    
  *    |  |     |  |  |  | |  . `  |     \   \       |  |     |      /     |  |  |  | |  |         |  |     |  |  |  | |      /     
  *    |  `----.|  `--'  | |  |\   | .----)   |      |  |     |  |\  \----.|  `--'  | |  `----.    |  |     |  `--'  | |  |\  \----.
  *     \______| \______/  |__| \__| |_______/       |__|     | _| `._____| \______/   \______|    |__|      \______/  | _| `._____|
  *                                                                                                                                 
  */    

  public RobotContainer() {
    configureDriverController();
    configureButtonBox();
    configureTestController();

    configureSmartDashboardButtons();
    configureDefaultCommands();
  }

  private void configureDriverController() {
    BooleanSupplier arcadeDriveForceLowGear = () -> m_driverController.getRightTrigger() > 0.5;
    DoubleSupplier arcadeDriveSpeedSupplier = DriveUtils.deadbandExponential(m_driverController::getLeftStickY,
        Constants.DRIVE_SPEED_EXP, Constants.DRIVE_JOYSTICK_DEADBAND);
    DoubleSupplier arcadeDriveCheesyDriveMinTurn = () -> arcadeDriveForceLowGear.getAsBoolean() ? Constants.CHEESY_DRIVE_FORCE_LOW_MIN_TURN : Constants.CHEESY_DRIVE_MIN_TURN;
    DoubleSupplier arcadeDriveCheesyDriveMaxTurn = () -> arcadeDriveForceLowGear.getAsBoolean() ? Constants.CHEESY_DRIVE_FORCE_LOW_MAX_TURN : Constants.CHEESY_DRIVE_MAX_TURN;
    DoubleSupplier arcadeDriveTurnSupplier = DriveUtils.cheesyTurn(
        arcadeDriveSpeedSupplier, DriveUtils.deadbandExponential(m_driverController::getRightStickX,
            Constants.DRIVE_SPEED_EXP, Constants.DRIVE_JOYSTICK_DEADBAND),
        arcadeDriveCheesyDriveMinTurn.getAsDouble(), arcadeDriveCheesyDriveMaxTurn.getAsDouble());
    BooleanSupplier arcadeDriveShiftSupplier = () -> !arcadeDriveForceLowGear.getAsBoolean() && m_drive.autoshift(arcadeDriveSpeedSupplier.getAsDouble());
    DoubleSupplier arcadeDriveMaxSpeedSupplier = () -> arcadeDriveForceLowGear.getAsBoolean() ? Constants.MAX_SPEED_LOW : Constants.MAX_SPEED_HIGH;
    m_arcadeDrive = new ArcadeDrive(m_drive, arcadeDriveSpeedSupplier, arcadeDriveTurnSupplier,
        arcadeDriveShiftSupplier, arcadeDriveMaxSpeedSupplier);
    SmartDashboard.putData("Arcade Drive", m_arcadeDrive);

    DoubleSupplier tankDriveLeftSupplier = m_driverController::getLeftStickY;
    DoubleSupplier tankDriveRightSupplier = m_driverController::getRightStickY;
    CommandBase m_tankDrive = new TankDrive(m_drive, tankDriveLeftSupplier, tankDriveRightSupplier);
    SmartDashboard.putData("Tank Drive", m_tankDrive);

    m_driverController.buttonA.whileHeld(new TurnToLimelight(m_drive, m_sensors));

    // m_driverController.buttonB.whileHeld(m_reportColor);
    // m_driverController.buttonA.whenPressed(m_moveColorWheelToTargetColor);
    // m_driverController.buttonY.whenPressed(m_rotateColorWheel);

    // m_driverController.buttonRightBumper.whenPressed(m_setToFrontCamera);
    // m_driverController.buttonRightBumper.whenReleased(m_setToRearCamera);
  }


  private void configureButtonBox() {
    m_buttonBox.button2.whenPressed(m_shoot);
    m_buttonBox.button2.whenPressed(new InstantCommand(() -> m_currentFASHCommand = m_shoot));
    m_buttonBox.button2.whenReleased(m_pauseShoot);
    m_buttonBox.button2.whenReleased(new InstantCommand(() -> m_currentFASHCommand = m_pauseShoot));
    m_buttonBox.button3.whenPressed(m_prepShoot);
    m_buttonBox.button3.whenPressed(new InstantCommand(() -> m_currentFASHCommand = m_prepShoot));
    m_buttonBox.button4.whenPressed(m_stopShoot);
    m_buttonBox.button4.whenPressed(new InstantCommand(() -> m_currentFASHCommand = m_stopShoot));
    m_buttonBox.button9.whenPressed(m_activateIntake);
    m_buttonBox.button9.whenReleased(m_deactivateIntake);
    m_buttonBox.button8.whenPressed(m_regurgitate);
    m_buttonBox.button8.whenReleased(m_regurgitateStop);
    m_buttonBox.button7.whenPressed(new InstantCommand(() -> {
      m_currentFASHCommand.cancel();
      m_currentFASHCommand.schedule();
    }));
    m_buttonBox.button7.whenReleased(new InstantCommand(() -> {
      m_currentFASHCommand.cancel();
      m_currentFASHCommand.schedule();
    }));
    m_buttonBox.button11.whenPressed(new ClimberMaxRobotHeight(m_climber));
    m_buttonBox.button14.whenPressed(new EngageRatchets(m_climber));
    m_buttonBox.button14.whenReleased(new DisengageRatchets(m_climber));
  }


  private void configureTestController() {
    SmartDashboard.putNumber("SetTestDriveSpeed", 0);
    SmartDashboard.putNumber("SetTestDriveTurn", 0);
    SmartDashboard.putBoolean("SetTestShiftHigh", false);
    BooleanSupplier testArcadeDriveShiftSupplier = () -> SmartDashboard.getBoolean("SetTestShiftHigh", false);
    DoubleSupplier testArcadeDriveSpeedSupplier = () -> SmartDashboard.getNumber("SetTestDriveSpeed", 0)
        / Constants.MAX_SPEED_HIGH;
    DoubleSupplier testArcadeDriveTurnSupplier = () -> SmartDashboard.getNumber("SetTestDriveTurn", 0)
        * (Constants.WHEEL_BASE_WIDTH * Math.PI) / Constants.MAX_SPEED_HIGH / 360;
    DoubleSupplier testArcadeDriveMaxSpeedSupplier = () -> Constants.MAX_SPEED_HIGH;
    m_testArcadeDrive = new ArcadeDrive(m_drive, testArcadeDriveSpeedSupplier, testArcadeDriveTurnSupplier,
        testArcadeDriveShiftSupplier, testArcadeDriveMaxSpeedSupplier);
    SmartDashboard.putData("TestArcadeDrive", m_testArcadeDrive);
    
    DoubleSupplier shooterSpeedSupplier = DriveUtils.deadbandExponential(m_testController::getLeftStickY,
        Constants.SHOOTER_FLYWHEEL_SPEED_EXP, Constants.TEST_JOYSTICK_DEADBAND);
    SmartDashboard.putData("Shooter Manual", new ShooterFlywheelRunBasic(m_shooter, shooterSpeedSupplier));

    DoubleSupplier armSpeedSupplier = DriveUtils.deadbandExponential(m_testController::getRightStickY,
        Constants.ARM_SPEED_EXP, Constants.TEST_JOYSTICK_DEADBAND);
    SmartDashboard.putData("Arm Manual", new RotateArm(m_arm, armSpeedSupplier));

  }


  private void configureSmartDashboardButtons() {
    // Drive testing
    SmartDashboard.putData("TestDriveStaticFriction", new TestDriveStaticFriction(m_drive));
    SmartDashboard.putData("CalculateDriveEfficiency", new CalculateDriveEfficiency(m_drive));
    SmartDashboard.putNumber("TurnTestHeading", 0);
    SmartDashboard.putData("TestTurnToHeading", new InstantCommand(() -> (new TurnToHeading(m_drive, m_sensors, SmartDashboard.getNumber("TurnTestHeading", 0))).schedule()));

    // Intake testing
    SmartDashboard.putData("Deploy Intake", new DeployIntake(m_intake));
    SmartDashboard.putData("Retract Intake", new RetractIntake(m_intake));
    SmartDashboard.putData("Run Intake", new RunIntake(m_intake, 1));
    SmartDashboard.putData("Stop Intake", new StopIntake(m_intake));
    SmartDashboard.putData("Eject Intake", new RunIntake(m_intake, -1));

    SmartDashboard.putNumber("Hopper Left Speed", 0);
    SmartDashboard.putNumber("Hopper Right Speed", 0);
    SmartDashboard.putData("Hopper Intake Mode", new HopperIntakeMode(m_hopper));
    SmartDashboard.putData("Hopper Shoot Mode", new HopperShootMode(m_hopper));
    SmartDashboard.putData("Hopper Stop", new HopperStop(m_hopper));
    SmartDashboard.putData("Hopper Test", new InstantCommand(() -> (new HopperTest(m_hopper, SmartDashboard.getNumber("Hopper Left Speed", 0), SmartDashboard.getNumber("Hopper Right Speed", 0))).schedule()));

    SmartDashboard.putData("Arm Calibrate", new CalibrateArm(m_arm));
    SmartDashboard.putNumber("ArmTestPosition", 0);
    SmartDashboard.putData("Arm to Position", new InstantCommand(() -> new ArmMotionMagic(m_arm, SmartDashboard.getNumber("ArmTestPosition", 0)).schedule()));
    SmartDashboard.putData("Arm to Stow", new ArmStow(m_arm, () -> m_driverController.getRawButton(6)));
    SmartDashboard.putData("Arm to Layup", new ArmMotionMagic(m_arm, m_armLayupAngle.getValue()));
    SmartDashboard.putData("Arm Hold Position", new InstantCommand(() -> new ArmMotionMagic(m_arm, m_arm.getCurrentArmPosition()).schedule(), m_arm));
    SmartDashboard.putData("Arm Test Brake Mode", new TestBrakeMode(m_arm));
    
    SmartDashboard.putNumber("FeederTestSpeed", 0);
    SmartDashboard.putData("FeederTest",new InstantCommand(() -> (new FeederRun(m_feeder, SmartDashboard.getNumber("FeederTestSpeed", 0))).schedule()));
    SmartDashboard.putData("FeederStop", new FeederStop(m_feeder));

    SmartDashboard.putNumber("ShooterTestFlywheelSpeed", 0);
    SmartDashboard.putData("ShooterTestFlywheel", new InstantCommand(() -> (new ShooterFlywheelRun(m_shooter, SmartDashboard.getNumber("ShooterTestFlywheelSpeed", 0))).schedule()));
    SmartDashboard.putData("ShooterStopFlywheel", new ShooterFlywheelRun(m_shooter, 0));
    
    SmartDashboard.putData("Regurgitate", m_regurgitate);
    SmartDashboard.putData("Regurgitate Stop", m_regurgitateStop);

    SmartDashboard.putData("Limelight On", new LimelightToggle(m_sensors, true));
    SmartDashboard.putData("Limelight Off", new LimelightToggle(m_sensors, false));

    SmartDashboard.putData("Zero Climber", new ZeroClimber(m_climber));

    SmartDashboard.putData("Engage Ratchets", new EngageRatchets(m_climber));
    SmartDashboard.putData("Disengage Ratchets", new DisengageRatchets(m_climber));

    SmartDashboard.putData("Turn To Limelight", new TurnToLimelight(m_drive, m_sensors));
    SmartDashboard.putData("Shoot Limelight", new ShooterRunFromLimelight(m_shooter));

    SmartDashboard.putData("Test Basic Auto 1", new BasicAutoDrive(m_drive, -10, -10, 8));
    SmartDashboard.putData("Test Basic Auto 2", new BasicAutoDrive(m_drive, -1, -6, 8));

    // Auto stuff
    SmartDashboard.putNumber("Auto Drive Wait", 10);
    SmartDashboard.putNumber("Auto Drive Distance", -2);
  }


  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(m_arcadeDrive);
    // m_arm.setDefaultCommand(m_armHoldCurrentPosition);
    m_intake.setDefaultCommand(new StopIntake(m_intake));
    m_hopper.setDefaultCommand(new HopperStop(m_hopper));
    m_feeder.setDefaultCommand(new FeederStop(m_feeder));
    m_shooter.setDefaultCommand(new ShooterStop(m_shooter));

    // DoubleSupplier climbSpeedXSupplier = m_buttonBox::getClimberTiltAxis;
    // DoubleSupplier climbSpeedYSupplier = m_buttonBox::getClimberSpeedAxis;  
    DoubleSupplier climbSpeedXSupplier = DriveUtils.deadbandExponential(m_buttonBox::getClimberTiltAxis, Constants.CLIMBER_EXPONENTIAL, Constants.CLIMBER_CONTROLLER_DEADZONE);
    DoubleSupplier climbSpeedYSupplier = DriveUtils.deadbandExponential(m_buttonBox::getClimberSpeedAxis, Constants.CLIMBER_EXPONENTIAL, Constants.CLIMBER_CONTROLLER_DEADZONE);
    m_climber.setDefaultCommand(new RunClimber(m_climber, climbSpeedXSupplier, climbSpeedYSupplier));
  }

  public void disabledPeriodic() {
    m_sensors.ledOff();

    for (ButtonAutoPair selector : autoSelectors) {
      selector.check();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
