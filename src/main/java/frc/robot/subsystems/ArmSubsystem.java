/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor = new CANSparkMax(Constants.ArmMotorCanId, MotorType.kBrushless);
  private CANEncoder armMotorEncoder;
  /**
   * Creates a new ExampleSubsystem.
   */
  public ArmSubsystem() {
    armMotorEncoder = armMotor.getEncoder();
  }

  public void openArm()
  {
    //Needs to rotate X degrees to open arm, there isn't a setPosition command, so either maintain power until an encoder value
    //is reached or use Position Closed Loop Control
  }

  public void stopped()
  {
    armMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
