package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
	CANcoder canCoder;
	TalonFX motor;
	TorqueCurrentFOC control;

	public ArmSubsystem() {
		canCoder = new CANcoder(CANCODER_PORT, "drivebase");
		motor = new TalonFX(MOTOR_PORT, "drivebase");

		SmartDashboard.putBoolean("Falcon is running", motor.isAlive());

		CANBus bus = new CANBus("drivebase");
		CANBus.CANBusStatus status = bus.getStatus();
		SmartDashboard.putString("CANivore status", status.toString());

		CANcoderConfiguration ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.MagnetOffset = 0.237539;
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		canCoder.getConfigurator().apply(ccConfig);

		TalonFXConfiguration talonConfig = new TalonFXConfiguration();
		talonConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
		talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
		talonConfig.Feedback.SensorToMechanismRatio = 1.0;
		talonConfig.Feedback.RotorToSensorRatio = 5.0;
		motor.getConfigurator().apply(talonConfig);

		control = new TorqueCurrentFOC(0);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("CANCoder angle", canCoder.getAbsolutePosition().getValueAsDouble() * 360);
		SmartDashboard.putNumber("Angle setpoint", Math.toDegrees(ARM_SETPOINT));
	}

	public double getAngle() {
		return canCoder.getPosition().getValueAsDouble() * Math.PI * 2.0;
	}

	public void run(double amps) {
		motor.setControl(control.withOutput(Units.Amps.of(amps / GEAR_RATIO)));

		SmartDashboard.putNumber("Falcon current", motor.getTorqueCurrent().getValueAsDouble());
	}
}
