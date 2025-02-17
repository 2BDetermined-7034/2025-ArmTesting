// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Elevator;

import java.io.File;

import static edu.wpi.first.units.Units.Volts;

public class RobotContainer {
	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandPS5Controller driverController = new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);
//	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	private final ArmSubsystem arm = new ArmSubsystem();
//	private final ArmSysID armSysID = new ArmSysID(0);
	private final Elevator elevatorSysID = new Elevator();
//	private final ElevatorSubsystem elevator = new ElevatorSubsystem();

	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
	}

	private void playIntro() {
		Orchestra orchestra = new Orchestra();

		TalonFX[] motors = new TalonFX[8];
		for (int i = 0; i < 8; ++i) {
			motors[i] = new TalonFX(i);
			orchestra.addInstrument(motors[i], i & 1);
		}

		var status = orchestra.loadMusic(new File(Filesystem.getDeployDirectory(), "intro.chrp").getPath());

		if (status.isOK()) {
			orchestra.play();
		}

		for(int i = 0; i < 8; i++) {
			motors[i].close();
		}
	}

	private void configureBindings() {
//		driverController.povUp().and(driverController.circle()).whileTrue(armSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//		driverController.povDown().and(driverController.circle()).whileTrue(armSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//		driverController.povUp().and(driverController.square()).whileTrue(armSysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
//		driverController.povDown().and(driverController.square()).whileTrue(armSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		driverController.povUp().and(driverController.circle()).whileTrue(elevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		driverController.povDown().and(driverController.circle()).whileTrue(elevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		driverController.povUp().and(driverController.square()).whileTrue(elevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
		driverController.povDown().and(driverController.square()).whileTrue(elevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		driverController.L2().whileTrue(arm.spinIntake(() -> Volts.of(driverController.getL2Axis() * 3), Constants.ArmConstants.IntakeType.INTAKE_CORAL));
		driverController.L1().whileTrue(elevatorSysID.setPosition(0.3));
		driverController.R1().whileTrue(elevatorSysID.setPosition(0.0));
		driverController.triangle().onTrue(elevatorSysID.zero());
		driverController.R2().whileTrue(elevatorSysID.upAndDown(() -> driverController.getR2Axis()));
	}

	public Command getAutonomousCommand() {
		return Commands.none();
	}
}
