/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class NavX {
    private final AHRS m_ahrs;

    public NavX() {
        m_ahrs = new AHRS(SPI.Port.kMXP);
    }

    public void zeroYaw() {
        m_ahrs.zeroYaw();
    }

    public double getYaw() {
        return m_ahrs.getYaw();
    }

    // 1-Jan-2020 Adding Pitch and Roll functions

    //Pitch is the slope of the line relative to horizontal.
    //Think of an airplane nose that is either up or down.
    public void zeroPitch() {
        //TBD
    }

    public double getPitch() {
        //TBD
    }

    //Roll is the rotation around the center axis.
    //Think of an airplane's wings moving around it's body.
    public void zeroRoll() {
        //TBD
    }
}
