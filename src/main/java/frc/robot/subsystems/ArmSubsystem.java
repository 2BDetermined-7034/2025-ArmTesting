package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
	CANcoder canCoder;
	SparkMax sparkMax;

	public ArmSubsystem() {
		canCoder = new CANcoder(CANCODER_PORT);
		sparkMax = new SparkMax(SPARK_MAX_PORT, SparkLowLevel.MotorType.kBrushless);
	}

	@Override
	public void periodic() {
		double voltage;
//		voltage = WEIGHT_NEWTONS * Math.cos()
//		sparkMax.setVoltage(voltage);
	}
}
