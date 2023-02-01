// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax rightMotor1;
  private final CANSparkMax rightMotor2;
  private final CANSparkMax leftMotor1;
  private final CANSparkMax leftMotor2;

  private MotorControllerGroup m_rightMotors;
  private DifferentialDrive m_drive;
  private MotorControllerGroup m_leftMotors;

  private AHRS m_gyro; // https://www.kauailabs.com/dist/frc/2022/navx_frc.json

  public DriveSubsystem() {
    rightMotor1 = new CANSparkMax(Constants.CANID.rightDeviceID1, MotorType.kBrushed);
    rightMotor2 = new CANSparkMax(Constants.CANID.rightDeviceID2, MotorType.kBrushed);
    leftMotor1 = new CANSparkMax(Constants.CANID.leftDeviceID1, MotorType.kBrushed);
    leftMotor2 = new CANSparkMax(Constants.CANID.leftDeviceID2, MotorType.kBrushed);

    m_leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
    m_rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

    m_leftMotors.setInverted(true);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    try {
      m_gyro = new AHRS(SerialPort.Port.kUSB);
      // m_gyro.setAngleAdjustment(180);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  public void periodic() {
    SmartDashboard.putString("Angle", m_gyro.getAngle() + "");
    SmartDashboard.putString("Roll", m_gyro.getRoll() + "");
    SmartDashboard.putString("Yaw", m_gyro.getYaw()+ "");
    SmartDashboard.putString("Pitch", m_gyro.getPitch()+ "");
    // SmartDashboard.putBoolean("At Setpoint", m_turnController.atSetpoint());
  }

  public void setLeftMotors(double speed) {
    SmartDashboard.putNumber("setLeftMotors", speed);
    m_leftMotors.set(speed);
  }

  public void setRightMotors(double speed) {
    SmartDashboard.putNumber("setRightMotors", speed);
    m_rightMotors.set(speed);
  }

  public void tankDrive(double left, double right) {
    SmartDashboard.putNumber("tankDrive:left", left);
    SmartDashboard.putNumber("tankDrive:right", right);
    m_drive.tankDrive(left, right);
  }

  public double getHeading() {
    if (m_gyro != null)
      return m_gyro.getYaw();
    return 0;
  }

  public double getPitch() {
    if (m_gyro != null)
      return m_gyro.getPitch();
    return 0;
  }

  public void arcadeDrive(double xSpeed, double zRotation, double limit) {
    m_drive.arcadeDrive(xSpeed, limit(zRotation, limit));
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    m_drive.arcadeDrive(xSpeed, zRotation);
  }

  public void reset() {
    if (m_gyro != null)
      m_gyro.reset();

  }

  public void resetGyro() {
    if (m_gyro != null) {
      System.out.println("Reset gyro");
      m_gyro.reset();
    }
  }

  private static double limit(double velocity, double limit) {
    if (velocity > limit)
      return limit;
    if (velocity < -limit)
      return -limit;
    return velocity;
  }

}
