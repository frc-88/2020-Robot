package frc.robot.driveutil;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Constants;

public class DriveConfiguration {
    public TJDriveModuleConfiguration left, right;

    private TalonFXConfiguration _talonMaster;
    private TalonFXConfiguration _talonFollower;

    public DriveConfiguration()
    {
        left = new TJDriveModuleConfiguration();
        right = new TJDriveModuleConfiguration();

        left.master = Constants.LEFT_MASTER_DRIVE_ID;
        left.talonFollowers = new int[] {Constants.LEFT_TALON_FOLLOWER_DRIVE_ID};
        
        right.master = Constants.RIGHT_MASTER_DRIVE_ID;
        right.talonFollowers = new int[] {Constants.RIGHT_TALON_FOLLOWER_DRIVE_ID};
        
        /* Master */

        _talonMaster = new TalonFXConfiguration();

        // TODO - properly configure TalonFX master
        // _talonMaster.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder; 
        // _talonMaster.neutralDeadband = 0.01;
        // _talonMaster.voltageCompSaturation = 12;

        left.masterConfiguration = _talonMaster;
        right.masterConfiguration = _talonMaster;

        // /* Followers */
        _talonFollower = new TalonFXConfiguration();

        // TODO - properly configure TalonFX follower
        // _talonFollower.neutralDeadband = 0.01;

        left.talonFollowerConfiguration = _talonFollower;
        right.talonFollowerConfiguration = _talonFollower;

        // /* General Settings */
        // left.neutralMode = NeutralMode.Brake;
        // left.invertMotor = true;
        // left.enableVoltageCompensation = true;
        // right.neutralMode = NeutralMode.Brake;
        // right.invertMotor = false;
        // right.enableVoltageCompensation = true;
    }
}