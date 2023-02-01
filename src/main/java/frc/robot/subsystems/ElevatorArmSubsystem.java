// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorArmSubsystem extends SubsystemBase {

  private final VictorSPX m_RightElevator;
  private final VictorSPX m_LeftElevator;
  private final VictorSPX m_RightArm;
  private final VictorSPX m_LeftArm;

  public ElevatorArmSubsystem() {

    m_RightElevator = new VictorSPX(6);
    m_LeftElevator = new VictorSPX(5);
    m_RightArm = new VictorSPX(8);
    m_LeftArm = new VictorSPX(7);

  }

  public void elevatorUp() {
    SmartDashboard.putBoolean("elevator", true);
    m_RightElevator.set(VictorSPXControlMode.PercentOutput, 0.5);
    m_LeftElevator.set(VictorSPXControlMode.PercentOutput, -0.5);

  }

  public void elevatorDown() {
    SmartDashboard.putBoolean("elevator", true);

    m_RightElevator.set(VictorSPXControlMode.PercentOutput, -0.4);
    m_LeftElevator.set(VictorSPXControlMode.PercentOutput, 0.4);

  }

  public void armFoward() {

    SmartDashboard.putBoolean("arm", true);
    m_RightArm.set(VictorSPXControlMode.PercentOutput, 0.7);
    m_LeftArm.set(VictorSPXControlMode.PercentOutput, -0.7);

  }

  public void elevatorStop() {

    SmartDashboard.putBoolean("elevator", false);
    m_RightElevator.set(VictorSPXControlMode.PercentOutput, 0);
    m_LeftElevator.set(VictorSPXControlMode.PercentOutput, 0);

  }

  public void armBackward() {

    SmartDashboard.putBoolean("arm", true);
    m_RightArm.set(VictorSPXControlMode.PercentOutput, -0.7);
    m_LeftArm.set(VictorSPXControlMode.PercentOutput, 0.7);

  }

  public void armStop() {

    SmartDashboard.putBoolean("arm", false);
    m_RightArm.set(VictorSPXControlMode.PercentOutput, 0);
    m_LeftArm.set(VictorSPXControlMode.PercentOutput, 0);

  }
}
