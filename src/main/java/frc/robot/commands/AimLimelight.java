package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AimLimelight extends PIDCommand {

   public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   private static double getX() {
    NetworkTableEntry tx = table.getEntry("tx");
    return tx.getDouble(0); 
   }

   public AimLimelight(DriveSubsystem driveSubsystem, boolean keepGoingIfNotDetected) {
       super(
               new PIDController(0.08, 0.03, 0.03),
               AimLimelight::getX,
               0,
               output -> {
                driveSubsystem.arcadeDrive(-0.5, output, 0.6);
               },
               driveSubsystem);
       getController().setTolerance(1, 10);
   }

   public AimLimelight(DriveSubsystem driveSubsystem) {
       this(driveSubsystem, true);
   }


   @Override
   public boolean isFinished() {
       return false;
   }

}
