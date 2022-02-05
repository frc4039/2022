package frc.robot.common;

import frc.robot.Constants;

public class Utilities {
	public static double deadband(double input) {
		return deadband(input, Constants.kDeadband);
	}

	public static double deadband(double input, double buffer) {
		input = (Math.abs(input) > Math.abs(buffer)) ? input : 0.0;
		if(input != 0.0)
			input = Math.signum(input) * ((Math.abs(input) - buffer) / (1.0 - buffer));

		return (Math.abs(input) > Math.abs(buffer)) ? input : 0.0;

		// Original version of deadband code from 2910 -- does not start at zero once buffer is passed
		// if (Math.abs(input) < buffer) return 0;
		// return input;
	}
}
