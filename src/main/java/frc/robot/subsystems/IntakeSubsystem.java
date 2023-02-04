// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Um botao pra abrir 1 lado do solenoide/ 1 pra a=fechar 2 lados 
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final DoubleSolenoid armLeft;
  private final DoubleSolenoid armRight;

  private final VictorSPX m_intakeLeft;
  private final VictorSPX m_intakeRight;

  public IntakeSubsystem() {
    
    armLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    armRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

    m_intakeLeft = new VictorSPX(9);
    m_intakeRight = new VictorSPX(10);
  }

  public void armLeftClose() {

    armLeft.set(DoubleSolenoid.Value.kReverse);

  }

  public void bothArmOpen() {

    armRight.set(DoubleSolenoid.Value.kForward);
    armLeft.set(DoubleSolenoid.Value.kForward);

  }

  public void bothArmClose() {

    armLeft.set(Value.kReverse);
    armRight.set(Value.kReverse);

  }

  public void intakeUp() {

    m_intakeLeft.set(VictorSPXControlMode.PercentOutput, Constants.kIntakeSpeed); // positivo
    m_intakeRight.set(VictorSPXControlMode.PercentOutput, -Constants.kIntakeSpeed);// negativo

  }

  public void intakeDown() {

    m_intakeLeft.set(VictorSPXControlMode.PercentOutput, -Constants.kIntakeSpeed); // negativo
    m_intakeRight.set(VictorSPXControlMode.PercentOutput, Constants.kIntakeSpeed); // positivo

  }

  public void intakeStop() {

    m_intakeLeft.set(VictorSPXControlMode.PercentOutput, 0);
    m_intakeRight.set(VictorSPXControlMode.PercentOutput, 0);

  }

  
  @Override
  public void periodic() {}

}

