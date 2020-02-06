package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeHopperShooterSubsystem;

public class AutoSimpleShoot extends SequentialCommandGroup
{
    public AutoSimpleShoot(IntakeHopperShooterSubsystem intakeHopperShooterSubsystem, DriveSubsystem DriveSubsystem)
    {
        addCommands(
            new ShootingTimedCommand(5.0, 1000, intakeHopperShooterSubsystem)
        );
    }
}

