// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSysID;

public class RobotContainer {
	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandPS4Controller driverController = new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);
	private ArmSysID armSysID = new ArmSysID();

	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		driverController.square().whileTrue(armSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		driverController.circle().whileTrue(armSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
	}

	public Command getAutonomousCommand() {
		return Commands.none();
	}
}
