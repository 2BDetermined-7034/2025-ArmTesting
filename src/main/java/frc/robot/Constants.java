// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int DRIVER_CONTROLLER_PORT = 0;
	}
	public static class ArmConstants {
		public static final int CANCODER_PORT = 12;
		public static final int MOTOR_PORT = 13;

		public static final double MASS = 1.165; // kg
		public static final double LENGTH = 0.908; // m
		public static final double G_ACCELERATION = 9.8067; // ms^-2
		public static final double DISTANCE_TO_COM = 0.449; // m

		public static final double WEIGHT_NEWTON_METERS = 9.76;
		public static final double KINETIC_FRICTION_NEWTON_METERS = 3.07;

		public static final double AMPS_PER_NEWTON_METER = 400.006 / 7.27;

		public static final double GEAR_RATIO = 80.0;
	}
}
