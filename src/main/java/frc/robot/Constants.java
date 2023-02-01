// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final int kLeftStickY = 5;
  public static final int kRightStickY = 1;
  public static final int kDriveJoystick = 1;
  public static final int kGameJoystick = 0; 
  public static final SerialPort.Port kGyroPort = SerialPort.Port.kUSB;
  public static final int kPneumaticsJoystick = 1;
  public static final int kPneumaticsSTART = 8;
  public static final int kPneumaticsRB = 5;
  public static final int kPneumaticsLB = 6;
  public static final int kBalanceButtonY = 4;
  public static final int kIntakeUpButtonY = 4;
  public static final int kIntakeDownButtonA = 1;
  public static final int kArmFowardButtonB = 2;
  public static final int kArmBackwardButtonX = 3;
  

  public class CANID{
    public static final int rightDeviceID1 = 3;
    public static final int rightDeviceID2 = 4;
    public static final int leftDeviceID1 = 1;
    public static final int leftDeviceID2 = 2;
  }

  public class Drive {
    public static final double kTurnP = 0.04;
    public static final double kTurnI = 0.06;
    public static final double kTurnD = 0.01;
    public static final double kBalanceP = 0.04;
    public static final double kBalanceI = 0.06;
    public static final double kBalanceD = 0.01;
    public static final double kToleranceDegrees = 5.0f;
    public static final double kTurnRateToleranceDegPerS = 10.0f;
    public static final double kEncoderLeftP = 0.03;
    public static final double kEncoderLeftI = 0.01;
    public static final double kEncoderLeftD = 0;
    public static final double kEncoderRightP = 0.03;
    public static final double kEncoderRightI = 0.01;
    public static final double kEncoderRightD = 0;
  }
}


