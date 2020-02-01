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
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;


public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterMotorCanId, MotorType.kBrushless);
  private CANPIDController pidController;

  /**
   * The power cells are shot by the shooter to score in the power 
   * 
   * To see how to do closed loop velocity control, see link below
   * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
   */
  public ShooterSubsystem() {
    // set PID coefficients (This may be need to be in robot init??)
    pidController.setP(Constants.ShooterkP);
    pidController.setI(Constants.ShooterkI);
    pidController.setD(Constants.ShooterkD);
    pidController.setIZone(Constants.ShooterkIz);
    pidController.setFF(Constants.ShooterkFF);
    pidController.setOutputRange(Constants.ShooterkMinOutput, Constants.ShooterkMaxOutput);

    pidController = shooterMotor.getPIDController();
  }

  /*
  */
  public void shoot(double targetVelocity) {
    pidController.setReference(targetVelocity, ControlType.kVelocity);
  }

  public void stopped() {
    shoot(0);
    shooterMotor.stopMotor();
  }
  
}
