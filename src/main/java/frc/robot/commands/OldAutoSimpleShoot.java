package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeHopperShooterSubsystem;

public class OldAutoSimpleShoot extends SequentialCommandGroup
{
    public OldAutoSimpleShoot(IntakeHopperShooterSubsystem intakeHopperShooterSubsystem, DriveSubsystem DriveSubsystem)
    {
        addCommands(
            new OldShootingTimedCommand(5.0, 1000, intakeHopperShooterSubsystem)
        );
    }
}

