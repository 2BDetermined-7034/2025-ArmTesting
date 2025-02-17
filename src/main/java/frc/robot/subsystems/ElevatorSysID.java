package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSysID extends SubsystemBase {
	private final SysIdRoutine routine;
	private final TalonFX motor;
	private VoltageOut voltageControl;

	public ElevatorSysID(int id) {
		this.motor = new TalonFX(id);

		Slot0Configs motorConfig = new Slot0Configs();
		motorConfig.kP = kP;
		motorConfig.kI = kI;
		motorConfig.kD = kD;

		motorConfig.kS = kS;
		motorConfig.kV = kV;
		motorConfig.kA = kA;
		motorConfig.kG = kG;

		motorConfig.GravityType = GravityTypeValue.Elevator_Static;
		motorConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

		motor.getConfigurator().apply(motorConfig);

		FeedbackConfigs ffConfig = new FeedbackConfigs();
		ffConfig.SensorToMechanismRatio = GEAR_RATIO;
		ffConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		ffConfig.FeedbackRotorOffset = 0.0;
		motor.getConfigurator().apply(ffConfig);

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
				(volts) -> {
					SmartDashboard.putNumber("Applied voltage", volts.in(Volts));
					motor.setControl(voltageControl.withOutput(volts));
				},
				(log) -> {
					log.motor("elevator")
						.voltage(motor.getMotorVoltage().getValue())
						.angularPosition(motor.getPosition().getValue())
						.angularVelocity(motor.getVelocity().getValue());
				},
				this
			)
		);
	}

	public void periodic() {
		SmartDashboard.putNumber("Elevator Position", motor.getPosition().getValue().in(Rotations) * (2.0 * Math.PI * SPOOL_RADIUS));
		SmartDashboard.putNumber("Elevator Voltage", motor.getMotorVoltage().getValue().in(Volts));
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}

	public Command setPosition(double position) {
		return new StartEndCommand(
			() -> motor.setControl(new PositionVoltage(position / (2.0 * Math.PI * SPOOL_RADIUS))),
			() -> motor.setControl(new PositionVoltage(motor.getPosition().getValue())),
			this
		);
	}
	public Command upAndDown(DoubleSupplier supplier){
		return new FunctionalCommand(
			() -> {},
			() -> {
				motor.setVoltage(supplier.getAsDouble());
			},
			(interrupted) -> {
				motor.setVoltage(0);
			},
			() -> {
				return false;
			},
			this

		);
	}

	public Command zero() {
		return Commands.runOnce(() -> {
			motor.setPosition(0);
		});
	}
}