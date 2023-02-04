package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class FindTarget extends PIDCommand {

    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private static double getX() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0);
    }

    public FindTarget(DriveSubsystem driveSubsystem, boolean keepGoingIfNotDetected) {
        super(
                new PIDController(0.08, 0.03, 0.03), // TODO: Ajustar valores de PID e criar constantes no arquivo
                                                     // Constants.java
                FindTarget::getX, // get do encoder
                0,
                output -> {
                    if (getA() > 0) {
                        driveSubsystem.arcadeDrive(-0.5, output, 0.6); // TODO: Ajustar valor xSpeed e limit
                    } else {
                        driveSubsystem.tankDrive(0.55, -0.55);
                    }
                },
                driveSubsystem);
        getController().setTolerance(1, 10);
    }

    public FindTarget(DriveSubsystem driveSubsystem) {
        this(driveSubsystem, true);
    }

    public static double getA() {
        NetworkTableEntry ta = table.getEntry("ta");
        SmartDashboard.putNumber("ta", ta.getDouble(-1));
        double area = ta.getDouble(0);
        return area;
    }

    @Override
    public boolean isFinished() { // TODO: comando deve terminar quando area > LIMITE, ajustar LIMITE
        return getA() > 0.1 && getController().atSetpoint();
    }

}
