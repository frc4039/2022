package frc.robot.common;

import frc.robot.Constants;

public class Utilities {
	public static double deadband(double input) {
		return deadband(input, Constants.kDeadband);
	}

	public static double deadband(double input, double buffer) {
		if (Math.abs(input) < buffer) return 0;
		return input;
	}
}
