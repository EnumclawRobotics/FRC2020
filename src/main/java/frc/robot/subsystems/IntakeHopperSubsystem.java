package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeHopperSubsystem extends SubsystemBase {
    //private CANSparkMax intakeMotor;
    private VictorSPX intakeMotor;
    //private CANSparkMax hopperMotor;
    private VictorSPX hopperMotor;

    public IntakeHopperSubsystem() {
        //intakeMotor = new CANSparkMax(Constants.IntakeMotorCanId, MotorType.kBrushless);
        intakeMotor = new VictorSPX(Constants.IntakeMotorCanId);
        //hopperMotor = new CANSparkMax(Constants.HopperMotorCanId, MotorType.kBrushless);
        hopperMotor = new VictorSPX(Constants.HopperMotorCanId);
    }

    public void intake() {
        //intakeMotor.set(Constants.IntakeIntakingPower);
        intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeIntakingPower);
        //hopperMotor.set(Constants.HopperIntakingPower);
        hopperMotor.set(ControlMode.PercentOutput, Constants.HopperIntakingPower);
    }

    public void expel() {
        //intakeMotor.set(Constants.IntakeExpellingPower);
        intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeExpellingPower);
        //hopperMotor.set(Constants.HopperExpellingPower);
        hopperMotor.set(ControlMode.PercentOutput, Constants.HopperExpellingPower);
    }

    public void transportToShooter() {
        //intakeMotor.set(Constants.IntakeShootingPower);
        intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeShootingPower);
        //hopperMotor.set(Constants.HopperShootingPower);
        hopperMotor.set(ControlMode.PercentOutput, Constants.HopperShootingPower);
    }

    public void stop() {
        //intakeMotor.stopMotor();
        intakeMotor.set(ControlMode.PercentOutput, 0);
        //hopperMotor.stopMotor();
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }
}