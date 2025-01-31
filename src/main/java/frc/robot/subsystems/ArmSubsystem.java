package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
	private TalonFX motor;
	private TorqueCurrentFOC control;
	private double setpoint = ARM_HOME_SETPOINT_RADIANS; // rad

	public ArmSubsystem() {
		motor = new TalonFX(MOTOR_PORT, "drivebase");
		motor.setPosition(ARM_HOME_SETPOINT_ROT);

		control = new TorqueCurrentFOC(0);

		SmartDashboard.putNumber("Arm Setpoint Radians", setpoint);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Arm Angle Degrees", getAngle() * (180/Math.PI));
		SmartDashboard.putNumber("Arm Angle Radians", getAngle());
		SmartDashboard.putNumber("Arm Angle Rotations", getAngle() / (2 * Math.PI) );
	}

	/**
	 *
	 * @return the arm's angle in radians
	 */
	public double getAngle() {
		return motor.getPosition().getValueAsDouble() * 2 * Math.PI / GEAR_RATIO;
	}

	public void run(double amps) {
		motor.setControl(control.withOutput(Units.Amps.of(amps / GEAR_RATIO)));
	}

	/**
	 * returns setpoitn in rad
	 * @return
	 */
	public double getSetpoint () {
		return setpoint;
	}

	public void setSetpoint (double setpoint) {
		this.setpoint = setpoint;
	}
}
