package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSysID extends SubsystemBase {
	CANcoder canCoder;
	TalonFX motor;
	SysIdRoutine sysIDRoutine;
	Voltage voltage;

	public ArmSysID() {
		canCoder = new CANcoder(CANCODER_PORT);
		motor = new TalonFX(MOTOR_PORT);

		CANcoderConfiguration ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.MagnetOffset = 0.0;
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		canCoder.getConfigurator().apply(ccConfig);

		TalonFXConfiguration talonConfig = new TalonFXConfiguration();
		talonConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
		talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
		talonConfig.Feedback.SensorToMechanismRatio = 1.0;
		talonConfig.Feedback.RotorToSensorRatio = 5.0;

		sysIDRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(),
			new SysIdRoutine.Mechanism(this::setVoltage, this::sysIDLog, this)
		);
	}

	public void setVoltage(Voltage volts) {
		motor.setVoltage(volts.in(Units.Volts));
		voltage = volts;
	}

	public void sysIDLog(SysIdRoutineLog routineLog) {
		routineLog.motor("Arm Motor").angularPosition(canCoder.getPosition().getValue());
		routineLog.motor("Arm Motor").angularVelocity(canCoder.getVelocity().getValue());
		routineLog.motor("Arm Motor").angularAcceleration(motor.getAcceleration().getValue());
		routineLog.motor("Arm Motor").voltage(voltage);
		routineLog.motor("Arm Motor").current(motor.getTorqueCurrent().getValue());
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIDRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIDRoutine.dynamic(direction);
	}
}
