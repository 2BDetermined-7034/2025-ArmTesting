package frc.robot.commands;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSysID extends SubsystemBase {
	private final SysIdRoutine routine;
	private final TalonFX motor;
	private VoltageOut voltageControl;

	public ElevatorSysID(int id) {
		motor = new TalonFX(id);

		voltageControl = new VoltageOut(0);

		var rampRate = Volts.of(8.0).div(Seconds.of(5));
		routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				rampRate,
				Volts.of(8.0),
				Seconds.of(10),
				null
			),
			new SysIdRoutine.Mechanism(
				(volts) -> motor.setControl(voltageControl.withOutput(volts.in(Volts))),
				(log) -> {
					log.motor("elevator")
						.voltage(motor.getMotorVoltage().getValue())
						.angularPosition(motor.getPosition().getValue().div(GEAR_RATIO).times(SPOOL_RADIUS))
						.angularVelocity(motor.getVelocity().getValue().div(GEAR_RATIO).times(SPOOL_RADIUS));
				},
				this
			)
		);
	}

	public void periodic() {
		SmartDashboard.putNumber("Elevator Angle", motor.getPosition().getValue().in(Radians) / GEAR_RATIO);
		SmartDashboard.putNumber("Elevator Voltage", motor.getMotorVoltage().getValue().in(Volts));
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}

	public Command aaa() {
		return Commands.run(
			() -> {
				motor.setControl(voltageControl.withOutput(Volts.of(5)));
			}
		);
	}
}