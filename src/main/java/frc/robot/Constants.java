/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Done
    public final static int FrontLeftMotorCanId = 9;
    public final static int BackLeftMotorCanId = 8;
    public final static int FrontRightMotorCanId = 7;
    public final static int BackRightMotorCanId = 6;
//    public final static boolean GyroReversed = false;

    public final static int IntakeMotorCanId = 10;      //VictorSPX - redline
    public final static double IntakeIntakingPower = .3;
    public final static double IntakeExpellingPower = -.3;
    public final static double IntakeShootingPower = .3;

    public final static int HopperMotorCanId = 11;        //VictorSPX - redline
    public final static double HopperIntakingPower = .3;
    public final static double HopperExpellingPower = -.3;
    public final static double HopperShootingPower = -.3;

    public final static int ShooterMotor1CanId = 12;    // VictorSPX - redline
    public final static int ShooterMotor2CanId = 13;    // VictorSPX - redline
    public final static int ShooterEncoderADIO = 1;     // Blue cable
    public final static int ShooterEncoderBDIO = 2;     // Yellow cable

    // PID coefficients
    public final static double ShooterkP = 5e-5; 
    public final static double ShooterkI = 1e-6;
    public final static double ShooterkD = 0; 
    public final static double ShooterkIz = 0; 
    public final static double ShooterkFF = 0; 
    public final static double ShooterkMaxOutput = 1; 
    public final static double ShooterkMinOutput = -1;
    public final static double ShootermaxRPS = 5700/60;
    public final static double ShooterkSVolts = 000;
    public final static double ShooterkWoltSecondsPerRotation = 000;
    public final static double ShooterTolerance = 000;
    public final static int ShooterEncoderCountsPerRevolution = 8192;
    public final static double ShooterFreeThrowRPS = 5000/60;

    public final static int PanelMotorCanId = 14; //Rotator, 14 Neo Brushless Brake
    public final static int FlipOutMotorCanId = 15; //(done) todo Physically put a Victor SPX based motor controller here
    public final static double MaxPanelMotorPower = 0.60;
    public final static I2C.Port ColorSensorPort = I2C.Port.kOnboard;

    public final static Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public final static Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public final static Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public final static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public final static int ArmMotorCanId = 16; //(done) todo Physically move Spark Max of Flipout to arm motor, Spark Max there is broken
    public final static double armMotorSpeed = 0.50;

    public final static int HookTraverseMotorCanId = 17; //Victor SPX
    public final static int HookLockServoChannel = 5;
    public final static double HookLockPosition = 0.5;
    public final static double HookUnlockPosition = 0.0;
    public final static double HookTraversePower = 1.0;

    public final static int WinchMotorCanId = 18; //Done
    public final static double winchSpeed = 0.5;

}
