/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

/**
 * An example command that uses a subsystem.
 */
public class ArmPositioningCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem subsystem;
  private int targetDegree;
  private int targetDirection;

  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
 public ArmPositioningCommand(int targetDegree, ArmSubsystem subsystem) {
   this.subsystem = subsystem;
   addRequirements(subsystem);
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetDirection = getDirectionToTarget();
  }

  private int getDirectionToTarget()
  {
      //Increase if less than target, decrease if more than target
      return (subsystem.getDegreesRotated() < targetDegree) ? 1 : -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      subsystem.moveArm(targetDirection * Constants.armMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      subsystem.stopped();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Should definately be finishing if passing target
    return (targetDirection == getDirectionToTarget()) ? false : true;
  }
}
