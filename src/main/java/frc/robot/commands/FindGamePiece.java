package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// Provavelmente nao sera usado
public class FindGamePiece extends PIDCommand {
    public FindGamePiece(DriveSubsystem drive) {
        super(
                new PIDController(0.03, 0.02, 0.02),  // TODO: Ajustar valores de PID e criar constantes no arquivo Constants.java
                RobotContainer.findGamePieceRunner::getCenterX,
                0,
                output -> {
                    if (RobotContainer.findGamePieceRunner.isDetected()) {
                        drive.arcadeDrive(0, output, 0.6);
                    } else {
                        drive.tankDrive(0.55, -0.55);
                    }
                },
                drive);
        getController().setTolerance(30, 30);
    }

    @Override
    public void execute() {
        System.out.println("FindGamePiece");
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}