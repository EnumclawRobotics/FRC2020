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

public class HopperSubsystem extends SubsystemBase {
  private CANSparkMax hopperMotor = new CANSparkMax(Constants.HopperMotorCanId, MotorType.kBrushless);

  /**
   * Hopper holds onto Power Cells with no spacing between.
   */
  public HopperSubsystem() {
  }

  /*
  */
  public void intaking() {
    hopperMotor.set(Constants.HopperIntakingPower);
  }

  /*
  */
  public void expelling() {
    hopperMotor.set(Constants.HopperExpellingPower);

  }

  public void stopped() {
    hopperMotor.stopMotor();
  }
}
