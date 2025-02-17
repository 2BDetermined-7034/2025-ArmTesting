package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
	private final TalonFX armMotor, intakeMotor;
	private final CANcoder armEncoder;

	/**
	 * <a href=https://www.chiefdelphi.com/t/using-sysid-with-torquefoc/454175></a>
	 */

	public ArmSubsystem() {
		this.armMotor = new TalonFX(ARM_MOTOR_ID);
		this.intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
		this.armEncoder = new CANcoder(CANCODER_ID);

		CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
		armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // TODO Change ::::::
		armEncoderConfig.MagnetSensor.MagnetOffset = 0.4;
		armEncoder.getConfigurator().apply(armEncoderConfig);

		TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
		armMotorConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
		armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
		armMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
		armMotorConfig.Feedback.RotorToSensorRatio = 10; // TODO CHANGE :::::
		armMotor.getConfigurator().apply(armMotorConfig);

		Slot0Configs armController = new Slot0Configs();
		armController.kP = kP;
		armController.kI = kI;
		armController.kD = kD;
		armController.kS = kS;
		armController.kG = kG;
		armController.kV = kV;
		armController.kA = kA;
		armMotor.getConfigurator().apply(armController);

	}

	@Override
	public void periodic() {

	}

	/**
	 * Sets the position of the arm
	 * @param position
	 * @return
	 */
	public Command setArmPosition(Angle position) {
		return new StartEndCommand(
			() -> armMotor.setControl(new PositionTorqueCurrentFOC(position)),
			() -> armMotor.setControl(new PositionTorqueCurrentFOC(armMotor.getPosition().getValue()))
		);
	}

	/**
	 * Spins the intake and ends the command once a current limit is reached
	 * @param supplier
	 * @return
	 */
	public Command spinIntake(Supplier<Voltage> supplier, IntakeType intakeType) {
		return new FunctionalCommand(
			() -> intakeMotor.setControl(new VoltageOut(supplier.get())),
			() -> {},
			(interrupted) -> intakeMotor.setControl(new VoltageOut(Volts.of(0))),
			() -> {return intakeMotor.getSupplyCurrent().getValue().abs(Amps) > intakeType.currentLimit.abs(Amps);},
			this
		);
	}


}
