// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
 
  private final DriveSubsystem m_DriveSubsystem;
  private final Joystick m_stickdrive;
  
  private  double rightStickY;
  private  double leftStickY;
  

  public DriveCommand(DriveSubsystem driveSubsystem, Joystick joystick) {
    m_DriveSubsystem = driveSubsystem;
    m_stickdrive = joystick;
    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    leftStickY = m_stickdrive.getRawAxis(Constants.kLeftStickY);
    rightStickY = m_stickdrive.getRawAxis(Constants.kRightStickY);
    m_DriveSubsystem.tankDrive(leftStickY, rightStickY);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;    
  }
}
