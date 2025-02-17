// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Current;

import static edu.wpi.first.units.Units.Amps;

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
		public static final int ARM_MOTOR_ID = 12;
		public static final int INTAKE_MOTOR_ID = 13;
		public static final int CANCODER_ID = 14;

		public static final Current INTAKE_CURRENT_LIMIT = Amps.of(10);

		public static final double kP = 0;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kV = 0;
		public static final double kA = 0;
		public static final double kG = 0;


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
