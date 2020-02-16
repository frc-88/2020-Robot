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
public class CPMConfig {
    public TalonFXConfiguration cpmConfiguration;

    public CPMConfig()
    {
        cpmConfiguration = new TalonFXConfiguration();

        // cpm - TalonFX
        cpmConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        // cpmConfiguration.primaryPID.selectedFeedbackCoefficient = 0.328293;
        cpmConfiguration.peakOutputForward = 1.0;
        cpmConfiguration.peakOutputReverse = -1.0;
        cpmConfiguration.nominalOutputForward = 1.0;
        cpmConfiguration.nominalOutputReverse = -1.0;
        cpmConfiguration.neutralDeadband = 0.001;
        // cpmConfiguration.slot0.kP = 0.00000;
        // cpmConfiguration.slot0.kI = 0.00000;
        // cpmConfiguration.slot0.kD = 0.00000;
        // cpmConfiguration.slot0.kF = 1.00000;
        // cpmConfiguration.slot0.integralZone = 900;
        // cpmConfiguration.slot0.allowableClosedloopError = 217;
        // cpmConfiguration.slot0.maxIntegralAccumulator = 254.000000;
        // cpmConfiguration.slot0.closedLoopPeakOutput = 0.869990;
        // cpmConfiguration.slot0.closedLoopPeriod = 33;
    }
}