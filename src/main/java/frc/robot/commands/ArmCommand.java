package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.*;

public class ArmCommand extends Command {
	private final ArmSubsystem arm;
	private final PIDController armPID = new PIDController(kP,kI,kD);

    public ArmCommand() {
		arm = new ArmSubsystem();
		addRequirements(arm);

	}

	@Override
	public void execute() {

		arm.setSetpoint(SmartDashboard.getNumber("Arm Setpoint Radians", ARM_HOME_SETPOINT_RADIANS));

		final double torque_ff = MASS * G_ACCELERATION * DISTANCE_TO_COM * Math.cos(arm.getAngle());

		double torque_feedback = armPID.calculate(arm.getAngle(), arm.getSetpoint());
		final double amps = torque_feedback * AMPS_PER_NEWTON_METER;
		arm.run(amps);



		SmartDashboard.putNumber("Arm Angle Degrees", arm.getAngle() * (180/Math.PI));
		SmartDashboard.putNumber("Arm Angle Radians", arm.getAngle());
		SmartDashboard.putNumber("Arm Angle Rotations", arm.getAngle() / (2 * Math.PI) );


	}


	@Override
	public void end(boolean interrupted) {
		arm.run(0.0);
		arm.setSetpoint(ARM_HOME_SETPOINT_RADIANS);
	}
}
