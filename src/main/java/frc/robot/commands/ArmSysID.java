package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.subsystems.ArmSubsystem;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import org.ejml.ops.DConvertMatrixStruct;
import sun.misc.Signal;

public class ArmSysID extends SubsystemBase {
	private final SysIdRoutine routine;
	private final TalonFX motor;
	private VoltageOut voltageControl;
	private SysIdRoutine.Direction direction;

	public ArmSysID(int id) {
		motor = new TalonFX(id, Constants.Misc.canBus);
		final Angle offset = Rotations.of(0.783203);

		voltageControl = new VoltageOut(0);

		var rampRate = Volts.of(1.0).div(Seconds.of(5));
		routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				rampRate,
				Volts.of(1.0),
				Seconds.of(10),
				null
			),
			new SysIdRoutine.Mechanism(
				(volts) -> motor.setControl(voltageControl.withOutput(volts.in(Volts))),
				(log) -> {
//					if (direction == SysIdRoutine.Direction.kReverse) {
//						log.motor("arm")
//							.voltage(Volts.of(-0.01))
//							.angularPosition(Radians.of(-0.01))
//							.angularVelocity(RadiansPerSecond.of(-0.01));
//					} else {
						log.motor("arm")
							.voltage(motor.getMotorVoltage().getValue())
							.angularPosition(motor.getPosition().getValue().plus(offset).div(GEAR_RATIO))
							.angularVelocity(motor.getVelocity().getValue());
//					}
				},
				this
			)
		);
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		this.direction = direction;
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		this.direction = direction;
		return routine.dynamic(direction);
	}
}
