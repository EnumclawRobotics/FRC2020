/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class WinchSubsystem extends SubsystemBase {
  private CANSparkMax winchMotor;
  private Servo clampServo;

  public WinchSubsystem() {
    winchMotor = new CANSparkMax(Constants.WinchMotorCanId, MotorType.kBrushless);
    clampServo = new Servo(Constants.WinchClampPwmId);
    clampServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    clampServo.setSpeed(1.0); // to open
  }

  public void setPower(double power)
  {
    winchMotor.set(power);
  }

  public void stop()
  {
    winchMotor.stopMotor();
  }

  public void clamp() {
    clampServo.setSpeed(-1.0); // to close
  }

  public void unclamp() {
    clampServo.setSpeed(1.0); // to open
  }
}
