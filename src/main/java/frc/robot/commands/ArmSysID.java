package frc.robot.commands;

import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ArmSubsystem;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;

public class ArmSysID extends SubsystemBase {
	private final SysIdRoutine routine;
	private final TalonFX motor;
	private VoltageOut voltageControl;

	public ArmSysID(int id) {
		motor = new TalonFX(id);

		voltageControl = new VoltageOut(0);

		var rampRate = Volts.of(0.7).div(Seconds.of(15));
		routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				rampRate,
				Volts.of(0.7),
				Seconds.of(15),
				(state) -> {}
			),
			new SysIdRoutine.Mechanism(
				(volts) -> motor.setControl(voltageControl.withOutput(volts.in(Volts))),
				null,
				this
			)
		);
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}
}
