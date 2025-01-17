package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.*;

public class ArmCommand extends Command {
	private final ArmSubsystem arm;

	public ArmCommand() {
		arm = new ArmSubsystem();
		addRequirements(arm);
	}

	@Override
	public void execute() {
		final double torque = MASS * G_ACCELERATION * DISTANCE_TO_COM * Math.cos(arm.getAngle());
		final double amps = torque * AMPS_PER_NEWTON_METER;
		arm.run(amps);
	}

	@Override
	public void end(boolean interrupted) {
		arm.run(0.0);
	}
}
