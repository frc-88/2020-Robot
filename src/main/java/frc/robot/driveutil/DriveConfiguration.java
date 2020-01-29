package frc.robot.driveutil;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Constants;

public class DriveConfiguration {
    public TJDriveModuleConfiguration left, right;

    private TalonFXConfiguration _masterConfiguration;
    private TalonFXConfiguration _followerConfiguration;

    public DriveConfiguration() {
        left = new TJDriveModuleConfiguration();
        right = new TJDriveModuleConfiguration();

        left.master = Constants.LEFT_MASTER_DRIVE_ID;
        left.followers = new int[] { Constants.LEFT_FOLLOWER_DRIVE_ID };

        right.master = Constants.RIGHT_MASTER_DRIVE_ID;
        right.followers = new int[] { Constants.RIGHT_FOLLOWER_DRIVE_ID };

        /* Master */

        _masterConfiguration = new TalonFXConfiguration();

        // TODO - properly configure TalonFX master
        // _talonMaster.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        // _talonMaster.neutralDeadband = 0.01;
        // _talonMaster.voltageCompSaturation = 12;

        left.masterConfiguration = _masterConfiguration;
        right.masterConfiguration = _masterConfiguration;

        // /* Followers */
        _followerConfiguration = new TalonFXConfiguration();

        // TODO - properly configure TalonFX follower
        // _talonFollower.neutralDeadband = 0.01;

        left.followerConfiguration = _followerConfiguration;
        right.followerConfiguration = _followerConfiguration;

        // /* General Settings */
        // left.neutralMode = NeutralMode.Brake;
         left.invertMotor = true;
        // left.enableVoltageCompensation = true;
        // right.neutralMode = NeutralMode.Brake;
         right.invertMotor = false;
        // right.enableVoltageCompensation = true;
    }
}