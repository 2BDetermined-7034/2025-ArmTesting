// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

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
	public static class Misc {
		public static final String canBus = "rio";
	}
	public static class ArmConstants {
		public static final int MOTOR_PORT = 12;

		public static final double MASS = 2.4494; // kg
		public static final double LENGTH = 0.320; // m
		public static final double G_ACCELERATION = 9.8067; // ms^-2
		public static final double DISTANCE_TO_COM = 0.07; // m

		public static final double AMPS_PER_NEWTON_METER = 476.10 / 9.36;

		public static final double GEAR_RATIO = 9 * (48d /36d);

		public static final double kP = 250.0 / AMPS_PER_NEWTON_METER;
		public static final double kI = 250.0 / AMPS_PER_NEWTON_METER;
		public static final double kD = 0.0 / AMPS_PER_NEWTON_METER;

		public static final double MAX_AMPS = 22.0;
		public static final double MOMENT_OF_INERTIA = 0.044;
		public static final double MAX_ANGULAR_ACCELERATION = GEAR_RATIO * MAX_AMPS / (AMPS_PER_NEWTON_METER * MOMENT_OF_INERTIA);
		public static final double MAX_ANGULAR_VELOCITY = 1.0;

		public static final double MOTOR_HOME_POSITION = (-48d / 360d) * GEAR_RATIO; //rot
		public static final double ARM_HOME_SETPOINT_RADIANS = MOTOR_HOME_POSITION * (2 * Math.PI) / GEAR_RATIO;
	}
	public static class ElevatorConstants {
		public static final int MOTOR_PORT = 0;

		public static final double MASS = 2.4494; // kg
		public static final double G_ACCELERATION = 9.8067; // ms^-2

		public static final double AMPS_PER_NEWTON_METER = 476.10 / 9.36;

		public static final double GEAR_RATIO = 20.0 * (48.0 / 24.0);
		public static final double SPOOL_RADIUS = 0.41275 / 80.8080462356;

		public static final double kP = 40;
		public static final double kI = 0.0;
		public static final double kD = 0.0; //0.70494

		public static final double kS = 0.233;
		public static final double kV = 4.9439;
		public static final double kA = 0.095073;
		public static final double kG = 0.031852;
	}
}
