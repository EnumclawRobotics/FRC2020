/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;

//To see how to do closed loop velocity control, see link below
//https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

public class IntakeHopperShooterSubsystem extends PIDSubsystem {
    private CANSparkMax intakeMotor;
    private CANSparkMax hopperMotor;
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private PIDController shooterPIDController;
    private CANEncoder shooterEncoder;
    private int shooterTargetRPM;

    public IntakeHopperShooterSubsystem()
    {
        intakeMotor = new CANSparkMax(Constants.IntakeMotorCanId, MotorType.kBrushless);
        hopperMotor = new CANSparkMax(Constants.HopperMotorCanId, MotorType.kBrushless);
        shooterMotor1 = new CANSparkMax(Constants.ShooterMotor1CanId, MotorType.kBrushed);
        shooterMotor2 = new CANSparkMax(Constants.ShooterMotor2CanId, MotorType.kBrushed);

        shooterPIDController = new PIDController(Constants.ShooterkP, Constants.ShooterkI, Constants.ShooterkD);

        //Constants.ShooterkP, Constants.ShooterkI, Constants.ShooterkD, Constants.ShooterkIz, Constants.ShooterkFF);
        //shooterPIDController.setOutputRange(Constants.ShooterkMinOutput, Constants.ShooterkMaxOutput);

        shooterEncoder = shooterMotor1.getEncoder();
    }

    public boolean isSpunUp() {
        return Math.abs(shooterEncoder.getVelocity() - shooterTargetRPM) < 10;
    }

    public void intake()
    {
        intakeMotor.set(Constants.IntakeIntakingPower);
        hopperMotor.set(Constants.HopperIntakingPower);
        setShooterTargetVelocity(0);
    }

    public void expel()
    {
        intakeMotor.set(Constants.IntakeExpellingPower);
        hopperMotor.set(Constants.HopperExpellingPower);
        setShooterTargetVelocity(0);
    }

    /**
     * @param targetVelocity (RPM)
     */
    public void shoot(final double targetVelocity)
    {
        setShooterTargetVelocity(targetVelocity);
        if (isSpunUp())
        {
            intakeMotor.set(Constants.IntakeShootingPower);
            hopperMotor.set(Constants.HopperShootingPower);
        }
        else {
            intakeMotor.set(0);
            hopperMotor.set(0);
        }
    }

    private void setShooterTargetVelocity(final double targetVelocity) 
    {
        shooterPIDController.setReference(targetVelocity, ControlType.kVelocity);
    }
    
    public void stopped() {
        intakeMotor.stopMotor();
        hopperMotor.stopMotor();
        setShooterTargetVelocity(0);
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }


}