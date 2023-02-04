// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ElevatorArmSubsystem;

public class PIDArmCommand extends PIDCommand {

  private final ElevatorArmSubsystem armSubsystem;


  public PIDArmCommand(ElevatorArmSubsystem armSubsystem, double setpoint) {
    super(
        new PIDController(0.1, 0.0, 0.0), 
        armSubsystem::getEncoderArm,
        setpoint,
        output -> {
          armSubsystem.setArmMotor(output);
        },
        armSubsystem
      );
    this.armSubsystem = armSubsystem;
  }

  @Override
  public boolean isFinished() {
      return super.isFinished();
  }
}
