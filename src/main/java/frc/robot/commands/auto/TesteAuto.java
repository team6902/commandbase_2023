package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ElevatorAutoCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TesteAuto extends SequentialCommandGroup {
 public TesteAuto(DriveSubsystem driveSubsystem, ElevatorArmSubsystem elevatorArmSubs, IntakeSubsystem intakeSubs) {
    addCommands(
    new DriveStraight(5000, driveSubsystem), // TODO: definir distancia pra charger station
      new TurnToAngle(90, driveSubsystem)
    );
}
}


  // elevator up com tempo
  // arm estende
  // intake dobra e solta 
  // d[a re]
//}
