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

public class WinchSubsystem extends SubsystemBase {
  private CANSparkMax winchMotor = new CANSparkMax(Constants.WinchMotorCanId, MotorType.kBrushless);
  /**
   * Creates a new ExampleSubsystem.
   */
  public WinchSubsystem() {

  }

  public void startWinching()
  {
    setPower(Constants.winchSpeed);
  }

  public void setPower(double power)
  {
    winchMotor.set(power);
  }

  public void stop()
  {
    winchMotor.stopMotor();
  }
}
