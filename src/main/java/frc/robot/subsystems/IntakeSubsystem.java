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

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeMotorCanId, MotorType.kBrushless);

  /**
   * Intake is used to collect power cells from the field
   */
  public IntakeSubsystem() {
  }

  /*
  */
  public void intaking() {
    intakeMotor.set(Constants.IntakeIntakingPower);
  }

  /*
  */
  public void expelling() {
    intakeMotor.set(Constants.IntakeExpellingPower);
  }

  public void stopped() {
    intakeMotor.stopMotor();
  }
}