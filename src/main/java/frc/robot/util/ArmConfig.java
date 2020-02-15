/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

/**
 * Add your docs here.
 */
public class ArmConfig {
    public TalonFXConfiguration armConfiguration;

    public ArmConfig()
    {
        armConfiguration = new TalonFXConfiguration();

        // arm - TalonFX
        // TODO - change to use Rev 1024 encoder on canifier
        armConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        // armConfiguration.primaryPID.selectedFeedbackCoefficient = 0.328293;
        armConfiguration.peakOutputForward = 1.0;
        armConfiguration.peakOutputReverse = -1.0;
        armConfiguration.nominalOutputForward = 1.0;
        armConfiguration.nominalOutputReverse = -1.0;
        armConfiguration.neutralDeadband = 0.001;
        armConfiguration.slot0.kP = 0.00000;
        armConfiguration.slot0.kI = 0.00000;
        armConfiguration.slot0.kD = 0.00000;
        armConfiguration.slot0.kF = 1.00000;
        // armConfiguration.slot0.integralZone = 900;
        // armConfiguration.slot0.allowableClosedloopError = 217;
        // armConfiguration.slot0.maxIntegralAccumulator = 254.000000;
        // armConfiguration.slot0.closedLoopPeakOutput = 0.869990;
        // armConfiguration.slot0.closedLoopPeriod = 33;
    }
}
