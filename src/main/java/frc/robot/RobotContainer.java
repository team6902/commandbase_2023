// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.AimLimelight;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

  private final Joystick m_stickdrive = new Joystick(Constants.kDriveJoystick);
  private final Joystick m_stickgame = new Joystick(Constants.kGameJoystick);
  private DriveCommand m_DriveCommand = new DriveCommand(m_DriveSubsystem, m_stickdrive);
  private final Balance m_Balance = new Balance(m_DriveSubsystem);
  private final JoystickButton m_BalanceButtonY = new JoystickButton(m_stickgame, Constants.kBalanceButtonY);

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ElevatorArmSubsystem m_ElevatorArmSubsystem = new ElevatorArmSubsystem();

  private final JoystickButton m_PneumaticsRB = new JoystickButton(m_stickgame, Constants.kPneumaticsRB);
  private final JoystickButton m_PneumaticsLB = new JoystickButton(m_stickgame, Constants.kPneumaticsLB);
  private final JoystickButton m_PneumaticsSTART = new JoystickButton(m_stickgame, Constants.kPneumaticsSTART);

  private final JoystickButton m_IntakeUpButtonY = new JoystickButton(m_stickgame, Constants.kIntakeUpButtonY);
  private final JoystickButton m_IntakeDownButtonA = new JoystickButton(m_stickgame, Constants.kIntakeDownButtonA);

  private final JoystickButton m_ArmFowardButtonB = new JoystickButton(m_stickgame, Constants.kArmFowardButtonB);
  private final JoystickButton m_ArmBacwardButtonX = new JoystickButton(m_stickgame, Constants.kArmBackwardButtonX);

  private final JoystickButton m_aimButton = new JoystickButton(m_stickdrive, 1);

  public RobotContainer() {

    configureBindings();
    CommandScheduler.getInstance().setDefaultCommand(m_DriveSubsystem, m_DriveCommand);

  }

  private void configureBindings() {

    m_aimButton.whileTrue(new AimLimelight(m_DriveSubsystem));

    m_BalanceButtonY.whileTrue(new Balance(m_DriveSubsystem));

    m_PneumaticsLB.onTrue(new InstantCommand(() -> m_IntakeSubsystem.armLeftClose(), m_IntakeSubsystem));
    m_PneumaticsSTART.onTrue(new InstantCommand(() -> m_IntakeSubsystem.bothArmOpen(), m_IntakeSubsystem));
    m_PneumaticsRB.onTrue(new InstantCommand(() -> m_IntakeSubsystem.bothArmClose(), m_IntakeSubsystem));

    m_IntakeUpButtonY.whileTrue(new StartEndCommand(() -> m_IntakeSubsystem.intakeUp(),
        () -> m_IntakeSubsystem.intakeStop(), m_IntakeSubsystem));
    m_IntakeDownButtonA.whileTrue(new StartEndCommand(() -> m_IntakeSubsystem.intakeDown(),
        () -> m_IntakeSubsystem.intakeStop(), m_IntakeSubsystem));

    m_ArmFowardButtonB.whileTrue(new StartEndCommand(() -> m_ElevatorArmSubsystem.armFoward(),
        () -> m_ElevatorArmSubsystem.armStop(), m_ElevatorArmSubsystem));
    m_ArmBacwardButtonX.whileTrue(new StartEndCommand(() -> m_ElevatorArmSubsystem.armBackward(),
        () -> m_ElevatorArmSubsystem.armStop(), m_ElevatorArmSubsystem));

    new POVButton(m_stickdrive, 0)
        .onTrue(new TurnToAngle(0, m_DriveSubsystem));
    new POVButton(m_stickdrive, 90)
        .onTrue(new TurnToAngle(90, m_DriveSubsystem));

    new POVButton(m_stickgame, 0)
        .whileTrue(new StartEndCommand(() -> m_ElevatorArmSubsystem.elevatorUp(),
            () -> m_ElevatorArmSubsystem.elevatorStop(), m_ElevatorArmSubsystem));
    new POVButton(m_stickgame, 180)
        .whileTrue(new StartEndCommand(() -> m_ElevatorArmSubsystem.elevatorDown(),
            () -> m_ElevatorArmSubsystem.elevatorStop(), m_ElevatorArmSubsystem));

  }

}