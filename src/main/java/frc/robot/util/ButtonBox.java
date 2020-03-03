/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBox extends Joystick {
	private static final int BUTTON_1 = 1;
	private static final int BUTTON_2 = 2;
	private static final int BUTTON_3 = 3;
	private static final int BUTTON_4 = 4;
	private static final int BUTTON_5 = 5;
	private static final int BUTTON_6 = 6;
	private static final int BUTTON_7 = 7;
	private static final int BUTTON_8 = 8;
	private static final int BUTTON_9 = 9;
	private static final int BUTTON_10 = 10;
	private static final int BUTTON_11 = 11;
	private static final int BUTTON_12 = 12;
	private static final int BUTTON_13 = 13;
	private static final int BUTTON_14 = 14;
	private static final int BUTTON_15 = 15;
	private static final int BUTTON_16 = 16;
	private static final int BUTTON_17 = 17;
	private static final int BUTTON_18 = 18;
	private static final int BUTTON_19 = 19;
	private static final int BUTTON_20 = 20;

	public JoystickButton button1 = new JoystickButton(this, BUTTON_1);
	public JoystickButton button2 = new JoystickButton(this, BUTTON_2);
	public JoystickButton button3 = new JoystickButton(this, BUTTON_3);
	public JoystickButton button4 = new JoystickButton(this, BUTTON_4);
	public JoystickButton button5 = new JoystickButton(this, BUTTON_5);
	public JoystickButton button6 = new JoystickButton(this, BUTTON_6);
	public JoystickButton button7 = new JoystickButton(this, BUTTON_7);
	public JoystickButton button8 = new JoystickButton(this, BUTTON_8);
	public JoystickButton button9 = new JoystickButton(this, BUTTON_9);
	public JoystickButton button10 = new JoystickButton(this, BUTTON_10);
	public JoystickButton button11 = new JoystickButton(this, BUTTON_11);
	public JoystickButton button12 = new JoystickButton(this, BUTTON_12);
	public JoystickButton button13 = new JoystickButton(this, BUTTON_13);
	public JoystickButton button14 = new JoystickButton(this, BUTTON_14);
	public JoystickButton button15 = new JoystickButton(this, BUTTON_15);
	public JoystickButton button16 = new JoystickButton(this, BUTTON_16);
	public JoystickButton button17 = new JoystickButton(this, BUTTON_17);
	public JoystickButton button18 = new JoystickButton(this, BUTTON_18);
	public JoystickButton button19 = new JoystickButton(this, BUTTON_19);
	public JoystickButton button20 = new JoystickButton(this, BUTTON_20);

	public ButtonBox(int port) {
		super(port);
	}

	public double getClimberSpeedAxis() {
		return -this.getRawAxis(1);
	}

	public double getClimberTiltAxis() {
		return this.getRawAxis(0);
	}
}