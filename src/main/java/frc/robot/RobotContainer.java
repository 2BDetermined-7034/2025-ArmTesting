// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSysID;

import java.io.File;

public class RobotContainer {
	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandPS4Controller driverController = new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);
	private ArmCommand armCommand = new ArmCommand();

	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		playIntro();
	}

	private void playIntro() {
		Orchestra orchestra = new Orchestra();

		TalonFX[] motors = new TalonFX[8];
		for (int i = 0; i < 8; ++i) {
			motors[i] = new TalonFX(i, "drivebase");
			orchestra.addInstrument(motors[i], i & 1);
		}

		var status = orchestra.loadMusic(new File(Filesystem.getDeployDirectory(), "intro.chrp").getPath());

		if (status.isOK()) {
			orchestra.play();
		}
	}

	private void configureBindings() {
//		driverController.square().whileTrue(armSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//		driverController.circle().whileTrue(armSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		driverController.cross().whileTrue(armCommand);
		new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(this::playIntro));
	}

	public Command getAutonomousCommand() {
		return Commands.none();
	}
}
