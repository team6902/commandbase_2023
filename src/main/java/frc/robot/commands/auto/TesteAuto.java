package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;

public class TesteAuto extends SequentialCommandGroup {
  public TesteAuto(DriveSubsystem driveSubsystem) {
    addCommands(
      new DriveStraight(1000, driveSubsystem),
      new TurnToAngle(90, driveSubsystem)
    );
  }

}
