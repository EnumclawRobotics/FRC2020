package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootingTimedCommand extends CommandBase {
    private IntakeHopperSubsystem intakeHopperSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private final Timer timer = new Timer();
    private double duration;
    private double shooterRPM;

    public ShootingTimedCommand(double duration, double shooterRPM, IntakeHopperSubsystem intakeHopperSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakeHopperSubsystem = intakeHopperSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.duration = duration;
        this.shooterRPM = shooterRPM;
        addRequirements(intakeHopperSubsystem, shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooterSubsystem.setSetpoint(shooterRPM);
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //TODO Learn how the useOutput command of a PIDSubsystem is used
        shooterSubsystem.useOutput(0, shooterRPM);

        if(shooterSubsystem.atSetpoint())
        {
            intakeHopperSubsystem.transportToShooter();
        }
        else
        {
            intakeHopperSubsystem.stopped();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeHopperSubsystem.stopped();
        shooterSubsystem.stopped();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (timer.get() >= duration);
    }
}