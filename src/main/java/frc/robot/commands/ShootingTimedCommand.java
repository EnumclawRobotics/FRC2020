package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootingTimedCommand extends CommandBase {
    private IntakeHopperSubsystem intakeHopperSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private final Timer timer = new Timer();
    private double duration;

    public ShootingTimedCommand(double duration, double shooterRPS, IntakeHopperSubsystem intakeHopperSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakeHopperSubsystem = intakeHopperSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.duration = duration;
        addRequirements(intakeHopperSubsystem, shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooterSubsystem.shoot();
        shooterSubsystem.enable();
        intakeHopperSubsystem.chargeIntakeForShooter();
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if(shooterSubsystem.atSetpoint())
        // {
        //     intakeHopperSubsystem.transportToShooter();
        // }
        // else
        // {
        //     intakeHopperSubsystem.stop();
        // }

        if (timer.get() > 3.5)
        {
            intakeHopperSubsystem.transportToShooter();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeHopperSubsystem.stop();
        shooterSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (timer.get() >= duration);
    }
}