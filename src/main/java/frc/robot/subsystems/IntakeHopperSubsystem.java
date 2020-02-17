package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeHopperSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private CANSparkMax hopperMotor;

    public IntakeHopperSubsystem() {
        intakeMotor = new CANSparkMax(Constants.IntakeMotorCanId, MotorType.kBrushless);
        hopperMotor = new CANSparkMax(Constants.HopperMotorCanId, MotorType.kBrushless);
    }

    public void intake() {
        intakeMotor.set(Constants.IntakeIntakingPower);
        hopperMotor.set(Constants.HopperIntakingPower);
    }

    public void expel() {
        intakeMotor.set(Constants.IntakeExpellingPower);
        hopperMotor.set(Constants.HopperExpellingPower);
    }

    public void transportToShooter() {
        intakeMotor.set(Constants.IntakeShootingPower);
        hopperMotor.set(Constants.HopperShootingPower);
    }

    public void stop() {
        intakeMotor.stopMotor();
        hopperMotor.stopMotor();
    }
}