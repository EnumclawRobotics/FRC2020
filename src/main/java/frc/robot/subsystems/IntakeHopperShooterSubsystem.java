/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

//To see how to do closed loop velocity control, see link below
//https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

public class IntakeHopperShooterSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeMotorCanId, MotorType.kBrushless);
    private CANSparkMax hopperMotor = new CANSparkMax(Constants.HopperMotorCanId, MotorType.kBrushless);
    private CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterMotorCanId, MotorType.kBrushless);
    private CANPIDController shooterPidController;
    private CANEncoder shooterEncoder;

    public IntakeHopperShooterSubsystem()
    {
        shooterPidController.setP(Constants.ShooterkP);
        shooterPidController.setI(Constants.ShooterkI);
        shooterPidController.setD(Constants.ShooterkD);
        shooterPidController.setIZone(Constants.ShooterkIz);
        shooterPidController.setFF(Constants.ShooterkFF);
        shooterPidController.setOutputRange(Constants.ShooterkMinOutput, Constants.ShooterkMaxOutput);

        shooterPidController = shooterMotor.getPIDController();
        shooterEncoder = shooterMotor.getEncoder();
    }

    public void intaking()
    {
        intakeMotor.set(Constants.IntakeIntakingPower);
        hopperMotor.set(Constants.HopperIntakingPower);
        shoot(0);
    }

    public void expelling()
    {
        intakeMotor.set(Constants.IntakeExpellingPower);
        hopperMotor.set(Constants.HopperExpellingPower);
        shoot(0);
    }

    /**
     * @param targetVelocity (RPM)
     */
    public void shooting(double targetVelocity)
    {
        shoot(targetVelocity);
        if (shooterEncoder.getVelocity() > 0.9 * targetVelocity)
        {
            intakeMotor.set(Constants.IntakeShootingPower);
            hopperMotor.set(Constants.HopperShootingPower);
        }
    
    }

    private void shoot(double targetVelocity) 
    {
        shooterPidController.setReference(targetVelocity, ControlType.kVelocity);
        
    }
    
    public void stopped() {
        intakeMotor.stopMotor();
        hopperMotor.stopMotor();
        shoot(0);
        shooterMotor.stopMotor();
    }


}