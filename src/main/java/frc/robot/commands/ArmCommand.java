package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.ArmConstants.*;

public class ArmCommand extends Command {
	private final ArmSubsystem arm;
	private final TrapezoidProfile profile;
	private final PIDController armPID = new PIDController(kP,kI,kD);

	private Timer timer = new Timer();

    public ArmCommand(ArmSubsystem arm) {
		profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
			MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION
		));

		armPID.setTolerance(0.01, 0.1);

		this.arm = arm;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		timer.restart();
	}

	@Override
	public void execute() {
		arm.setSetpoint(SmartDashboard.getNumber("Arm Setpoint Radians", ARM_HOME_SETPOINT_RADIANS));
		final var curState = new TrapezoidProfile.State(arm.getAngle(), arm.getAngularVelocity());
		final var endState = new TrapezoidProfile.State(arm.getSetpoint(), 0.0);

		final double torqueFeedForward = MASS * G_ACCELERATION * DISTANCE_TO_COM * Math.cos(arm.getAngle());

//		final TrapezoidProfile.State targetState = profile.calculate(timer.get(), curState, endState);
		final double torquePID = armPID.calculate(arm.getAngle(), arm.getSetpoint());

		final double torque = torqueFeedForward + torquePID;
		final double amps = MathUtil.clamp(torque * AMPS_PER_NEWTON_METER / GEAR_RATIO, -MAX_AMPS, MAX_AMPS);
		SmartDashboard.putNumber("Arm Output (A)", amps);
		SmartDashboard.putNumber("Arm torque (Nm)", torque);
		arm.run(amps);
	}


	@Override
	public void end(boolean interrupted) {
		arm.run(0.0);
		arm.setSetpoint(ARM_HOME_SETPOINT_RADIANS);
	}
}
