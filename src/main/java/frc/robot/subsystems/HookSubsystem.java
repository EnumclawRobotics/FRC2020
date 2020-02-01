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

import edu.wpi.first.wpilibj.Servo;

public class HookSubsystem extends SubsystemBase {
  //Plan to actually use: Johnson Electric Gearmotor and Output Shaft am-4230, which is a PWM based motor,
  //may need to be specially configured for its deadzones and such to get it working.
  private CANSparkMax hookTraverseMotor = new CANSparkMax(Constants.HookTraverseMotorCanId, MotorType.kBrushed);
  private Servo hooklockServo = new Servo(Constants.HookLockServoChannel);

  /**
   * Creates a new ExampleSubsystem.
   */
  public HookSubsystem() {

  }

  public void traversePower(double power)
  {
    hookTraverseMotor.set(power);
  }

  public void lockHook()
  {
    hooklockServo.setPosition(Constants.HookLockPosition);
  }

  public void unlockHook()
  {
    hooklockServo.setPosition(Constants.HookUnlockPosition);
  }

  public void stopped()
  {
    hooklockServo.stopMotor();
    hookTraverseMotor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
