package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Map;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
	private final TalonFX masterMotor;
	private final TalonFX slaveMotor;
	private final CANcoder canCoder;

	private final SysIdRoutine routine;

	public Elevator() {
		masterMotor = new TalonFX(MASTER_MOTOR_ID);
		slaveMotor = new TalonFX(SLAVE_MOTOR_ID);
		canCoder = new CANcoder(CANCODER_ID);

		slaveMotor.setControl(new Follower(MASTER_MOTOR_ID, true));

		CANcoderConfiguration ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		ccConfig.MagnetSensor.MagnetOffset = 0.0;
		canCoder.getConfigurator().apply(ccConfig);

		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
		motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
		motorConfig.Feedback.SensorToMechanismRatio = SPOOL_RADIUS;
		motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
		masterMotor.getConfigurator().apply(motorConfig);

		var stepCurrent = Volts.of(10);
		var testDuration = Seconds.of(5);
		var rampRate = stepCurrent.div(testDuration);

		routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				rampRate,
				stepCurrent,
				testDuration
			),
			new SysIdRoutine.Mechanism(
				(amps) -> {
					if (amps.in(Volts) > MAX_CURRENT) {
						masterMotor.setControl(new TorqueCurrentFOC(amps.in(Volts)));
					}
				},
				(log) -> {
					log.motor("elevator")
						.current(masterMotor.getTorqueCurrent().getValue())
						.angularPosition(masterMotor.getPosition().getValue())
						.angularVelocity(masterMotor.getVelocity().getValue());
				},
				this
			)
		);
	}

	public void periodic() {
		SmartDashboard.putNumber("Elevator Position", masterMotor.getPosition().getValue().in(Rotations) * (2.0 * Math.PI * SPOOL_RADIUS));
		SmartDashboard.putNumber("Elevator Current", masterMotor.getTorqueCurrent().getValue().in(Amps));
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}

	public Command setPosition(double position) {
		return new StartEndCommand(
			() -> masterMotor.setControl(new PositionTorqueCurrentFOC(position)),
			() -> masterMotor.setControl(new PositionTorqueCurrentFOC(masterMotor.getPosition().getValue())),
			this
		);
	}

	public Command upAndDown(DoubleSupplier supplier){
		return new FunctionalCommand(
			() -> {},
			() -> {
				masterMotor.setVoltage(supplier.getAsDouble());
			},
			(interrupted) -> {
				masterMotor.setVoltage(0);
			},
			() -> {
				return false;
			},
			this

		);
	}

	public Command zero() {
		return Commands.runOnce(() -> {
			canCoder.setPosition(0);
			masterMotor.setPosition(0);
		});
	}
}