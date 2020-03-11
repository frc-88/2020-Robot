/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sensors;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LimelightToggle extends InstantCommand {
  private Sensors sensors;
  private boolean toggle;
  public LimelightToggle(Sensors sensors, boolean toggle) {
    this.sensors=sensors;
    this.toggle=toggle;
    addRequirements(sensors);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(toggle) {
      sensors.ledOn();
    } else {
      sensors.ledOff();
    }
  }
}
