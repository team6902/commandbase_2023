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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    leftStickY = m_stickdrive.getRawAxis(Constants.kLeftStickY);
    rightStickY = m_stickdrive.getRawAxis(Constants.kRightStickY);
    m_DriveSubsystem.tankDrive(leftStickY, rightStickY);
    // m_DriveSubsystem.setRightMotors(rightStickY);
    // m_DriveSubsystem.setLeftMotors(leftStickY);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;    
  }
}
