/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeHopperShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

/**
 * An example command that uses a subsystem.
 */
public class ShootingTimedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeHopperShooterSubsystem subsystem;
  private final Timer timer = new Timer();
  private double duration;
  private double shooterRMP;

  /**
   * Creates a new Command.
   *
   * @param subsystem The subsystem used by this command.
   */
 public ShootingTimedCommand(double duration, double shooterRPM, IntakeHopperShooterSubsystem subsystem) {
   this.subsystem = subsystem;
   this.duration = duration;
   this.shooterRMP = shooterRPM;
   addRequirements(subsystem);
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      timer.reset();
      timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.shoot(shooterRMP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      subsystem.stopped();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= duration);
  }
}
