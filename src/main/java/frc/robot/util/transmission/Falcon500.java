/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.transmission;

/**
 * Contains motor characteristic information 
 * for the Falcon500 brushless motor.
 */
public class Falcon500 implements Motor{

    @Override
    public double getVelocityConstant() {
        // TODO
        return 0;
    }

    @Override
    public double getWindingsResistance() {
        // TODO
        return 0;
    }
}
