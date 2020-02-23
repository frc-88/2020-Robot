/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class ShooterConfig {
    public TalonFXConfiguration flywheelConfiguration;

    public ShooterConfig()
    {
        flywheelConfiguration = new TalonFXConfiguration();

        // flywheel - TalonFX
        flywheelConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        // flywheelConfiguration.primaryPID.selectedFeedbackCoefficient = 0.328293;
        flywheelConfiguration.peakOutputForward = 1.0;
        flywheelConfiguration.peakOutputReverse = -1.0;
        flywheelConfiguration.nominalOutputForward = 0;
        flywheelConfiguration.nominalOutputReverse = 0;
        flywheelConfiguration.neutralDeadband = 0.001;
        // flywheelConfiguration.slot0.integralZone = 900;
        // flywheelConfiguration.slot0.allowableClosedloopError = 217;
        // flywheelConfiguration.slot0.maxIntegralAccumulator = 254.000000;
        // flywheelConfiguration.slot0.closedLoopPeakOutput = 0.869990;
        // flywheelConfiguration.slot0.closedLoopPeriod = 33;
        flywheelConfiguration.voltageCompSaturation = 12;
    }
}