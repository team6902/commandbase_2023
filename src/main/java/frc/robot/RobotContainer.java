// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.AimGamePiece;
import frc.robot.commands.AimLimelight;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.FindGamePiece;
import frc.robot.commands.PIDArmCommand;
import frc.robot.commands.TurnToAngle;
//import frc.robot.commands.auto.TesteAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.FindGamePieceRunner;
import frc.robot.vision.GamePieceColor;

public class RobotContainer {
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  public static final FindGamePieceRunner findGamePieceRunner = new FindGamePieceRunner(GamePieceColor.PURPLE);
  private Thread findGamePieceThread = new Thread(findGamePieceRunner);
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final Joystick m_stickdrive = new Joystick(Constants.kDriveJoystick);
  private final Joystick m_stickgame = new Joystick(Constants.kGameJoystick);
  private DriveCommand m_DriveCommand = new DriveCommand(m_DriveSubsystem, m_stickdrive);
  private final Balance m_Balance = new Balance(m_DriveSubsystem);
  private final JoystickButton m_BalanceButtonBACK= new JoystickButton(m_stickgame, Constants.kBalanceButtonBACK);

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ElevatorArmSubsystem m_ElevatorArmSubsystem = new ElevatorArmSubsystem();

  private final JoystickButton m_PneumaticsRB = new JoystickButton(m_stickgame, Constants.kPneumaticsRB);
  private final JoystickButton m_PneumaticsLB = new JoystickButton(m_stickgame, Constants.kPneumaticsLB);
  private final JoystickButton m_PneumaticsSTART = new JoystickButton(m_stickgame, Constants.kPneumaticsSTART);
  
  private final JoystickButton m_ArmFowardButtonB = new JoystickButton(m_stickgame, Constants.kArmFowardButtonB);
  private final JoystickButton m_ArmBacwardButtonX = new JoystickButton(m_stickgame, Constants.kArmBackwardButtonX);

  private final JoystickButton m_IntakeUpButtonY = new JoystickButton(m_stickgame, Constants.kIntakeUpButtonY);
  private final JoystickButton m_IntakeDownButtonA = new JoystickButton(m_stickgame, Constants.kIntakeDownButtonA);

  private final JoystickButton m_aimButton = new JoystickButton(m_stickdrive, 1);
  private final JoystickButton m_detectGamePieceButton = new JoystickButton(m_stickdrive, 2);
  private final JoystickButton m_findCargoPurpleButton = new JoystickButton(m_stickdrive, 3);
  private final JoystickButton m_findCargoYellowButton = new JoystickButton(m_stickdrive, 4);

 
  public RobotContainer() {
    

    configureBindings();
    
    m_autoChooser.setDefaultOption("1 - [AUTO MAIN]", new DriveStraight(1000, m_DriveSubsystem)); 
   // m_autoChooser.addOption("1 - [TEST] Auto", new TesteAuto(m_DriveSubsystem));
    // m_autoChooser.addOption("10 - [TEST] Drive forward", auto);

    SmartDashboard.putData(m_autoChooser);
    CommandScheduler.getInstance().setDefaultCommand(m_DriveSubsystem, m_DriveCommand);
    findGamePieceThread.start();
  }

  public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();
  }

  private void configureBindings() {

    m_aimButton.whileTrue(new AimLimelight(m_DriveSubsystem));
    m_findCargoPurpleButton
        .whileTrue(new InstantCommand(() -> findGamePieceRunner.setGamePieceColor(GamePieceColor.PURPLE))
            .andThen(new AimGamePiece(m_DriveSubsystem)));
    m_findCargoYellowButton
        .whileTrue(new InstantCommand(() -> findGamePieceRunner.setGamePieceColor(GamePieceColor.YELLOW))
            .andThen(new AimGamePiece(m_DriveSubsystem)));
    m_detectGamePieceButton
        .whileTrue(new InstantCommand(() -> findGamePieceRunner.setGamePieceColor(null))
            .andThen(new AimGamePiece(m_DriveSubsystem)));

    m_BalanceButtonBACK.whileTrue(new Balance(m_DriveSubsystem));

    // m_ArmFowardButton.whileTrue(new PIDArmCommand(m_ElevatorArmSubsystem, VALOR_A_DEFINIR));

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

    new POVButton(m_stickgame, 0)
        .whileTrue(new StartEndCommand(() -> m_ElevatorArmSubsystem.elevatorUp(),
            () -> m_ElevatorArmSubsystem.elevatorStop(), m_ElevatorArmSubsystem));
    new POVButton(m_stickgame, 180)
        .whileTrue(new StartEndCommand(() -> m_ElevatorArmSubsystem.elevatorDown(),
            () -> m_ElevatorArmSubsystem.elevatorStop(), m_ElevatorArmSubsystem));

    new POVButton(m_stickdrive, 0)
        .onTrue(new TurnToAngle(0, m_DriveSubsystem));
    new POVButton(m_stickdrive, 90)
        .onTrue(new TurnToAngle(90, m_DriveSubsystem));

  }
}