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

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private double power;

  private enum ArmState {
    Stopped, Reaching, Retracting
  }

  private ArmState state = ArmState.Stopped;

  /**
   * Creates a new ExampleSubsystem.
   */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.ArmMotorCanId, MotorType.kBrushless);
    armMotor.setInverted(true);
    armMotor.setOpenLoopRampRate(1);

    
//    armMotorEncoder = armMotor.getEncoder();
    // store degrees per click so that getPosition returns degrees
//    armMotorEncoder.setPositionConversionFactor(360); // what gearbox?
  }

  // // In degrees due to conversion factor
  // public double getRotation() {
  //   return armMotorEncoder.getPosition();
  // }

  // public void zeroPosition() {
  //   armMotorEncoder.setPosition(0);
  // }

  // public void setSetpoint(double setpoint) {
  //   if (setpoint > getRotation()) {
  //     this.state = ArmState.Reaching;
  //   }
  //   else if (setpoint < getRotation()) {
  //     this.state = ArmState.Retracting;
  //   }
  //   else {
  //     this.state = ArmState.Stopped;
  //   }

  //   this.setpoint = setpoint;
  // }

  public void setPower(double power) {
    if (power < 0) {
      state = ArmState.Reaching;
    }
    else if (power > 0) {
      state = ArmState.Retracting;
    }
    else {
      state = ArmState.Stopped;
    }

    this.power = power;
  }

  public void stop() {
    this.state = ArmState.Stopped;
    this.power = 0;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    if (this.power != 0) {
      if (this.state == ArmState.Reaching) {
        armMotor.set(this.power * Constants.ArmReachPowerSpan);
      } else if (this.state == ArmState.Retracting) {
        armMotor.set(this.power * Constants.ArmRetractPowerSpan);
      }
    } else {
        armMotor.stopMotor();
    }
  }

  // @Override
  // public void periodic() {
  //   // //This assumes positive power is reaching and positive encoder values is reaching
  //   // if (state == ArmState.Reaching && setpoint > getRotation()) {
  //   //   armMotor.set(1);
  //   // }
  //   // else if (state == ArmState.Retracting && setpoint < getRotation()) {
  //   //   armMotor.set(-1);
  //   // }
  //   // else {
  //   //   armMotor.stopMotor();
  //   // }
  // }
}