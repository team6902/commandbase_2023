// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorArmSubsystem extends SubsystemBase {

  private final VictorSPX m_RightElevator;
  private final VictorSPX m_LeftElevator;
  private final VictorSPX m_RightArm;
  private final VictorSPX m_LeftArm;
  private final double kDriveTick2Feet = 1.0/128 * 6 *Math.PI/12;
  public Encoder encoderArmL;
  public Encoder encoderArmR;

  public ElevatorArmSubsystem() {

    m_LeftElevator = new VictorSPX(5);
    m_RightElevator = new VictorSPX(6);
    m_RightArm = new VictorSPX(8);
    m_LeftArm = new VictorSPX(7);
    encoderArmL = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    encoderArmR = new Encoder(2, 3, true, EncodingType.k4X);
    encoderArmL.setDistancePerPulse((3.141592*5)/360);
    encoderArmR.setDistancePerPulse((3.141592*5)/360);


  }

  public void elevatorUp() {
    SmartDashboard.putBoolean("elevator", true);
    m_RightElevator.set(VictorSPXControlMode.PercentOutput, Constants.kElevadorSpeed); // positivo
    m_LeftElevator.set(VictorSPXControlMode.PercentOutput, -Constants.kElevadorSpeed); // negativo

  }

  public void elevatorDown() {
    SmartDashboard.putBoolean("elevator", true);

    m_RightElevator.set(VictorSPXControlMode.PercentOutput, -Constants.kElevadorSpeed); // negativo
    m_LeftElevator.set(VictorSPXControlMode.PercentOutput, Constants.kElevadorSpeed); // positivo

  }

  public void elevatorStop() {

    SmartDashboard.putBoolean("elevator", false);
    m_RightElevator.set(VictorSPXControlMode.PercentOutput, 0);
    m_LeftElevator.set(VictorSPXControlMode.PercentOutput, 0);

  }

  public void armFoward() {

    SmartDashboard.putBoolean("arm", true);
    m_RightArm.set(VictorSPXControlMode.PercentOutput, Constants.kArmSpeedFoward); // positivo
    m_LeftArm.set(VictorSPXControlMode.PercentOutput, -Constants.kArmSpeedBackward); // negativo
    encoderArmL.reset();
    encoderArmR.reset();

  }

  public void armBackward() {

    SmartDashboard.putBoolean("arm", true);
    m_RightArm.set(VictorSPXControlMode.PercentOutput, -Constants.kArmSpeedBackward); // negativo
    m_LeftArm.set(VictorSPXControlMode.PercentOutput, Constants.kArmSpeedBackward); // positivo

  }

  public void armStop() {

    SmartDashboard.putBoolean("arm", false);
    m_RightArm.set(VictorSPXControlMode.PercentOutput, 0);
    m_LeftArm.set(VictorSPXControlMode.PercentOutput, 0);

  }

  public void setArmMotor(double speed) {
    m_RightArm.set(VictorSPXControlMode.PercentOutput, speed);
    m_LeftArm.set(VictorSPXControlMode.PercentOutput, -speed);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder ArmL", encoderArmL.get() *kDriveTick2Feet);
    SmartDashboard.putNumber("Encoder ArmR", encoderArmR.get() *kDriveTick2Feet);

  }

  public double getEncoderArm() {
      return encoderArmL.getRaw(); // TODO: Ajustar para cm ou testar valor raw
  }
}
