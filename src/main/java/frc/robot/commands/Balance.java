package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance extends PIDCommand {

    private DriveSubsystem m_driveSubsystem;

    public Balance(DriveSubsystem driveSubsystem) {
        super(new PIDController(Constants.Drive.kBalanceP, Constants.Drive.kBalanceI, Constants.Drive.kBalanceD),
                driveSubsystem::getPitch,
                0,
                output -> driveSubsystem.arcadeDrive(output/2, 0),
                driveSubsystem);
        
        this.m_driveSubsystem = driveSubsystem;
        getController()
                .setTolerance(Constants.Drive.kToleranceDegrees, Constants.Drive.kTurnRateToleranceDegPerS);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.tankDrive(0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}