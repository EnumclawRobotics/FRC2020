
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;

public class ShooterSubsystem extends PIDSubsystem {
    
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private CANEncoder encoder1;
    private SimpleMotorFeedforward shooterFeedforward;

    public ShooterSubsystem() {
        super(new PIDController(Constants.ShooterkP, Constants.ShooterkI, Constants.ShooterkD));
        
        motor1 = new CANSparkMax(Constants.ShooterMotor1CanId, MotorType.kBrushed);
        motor2 = new CANSparkMax(Constants.ShooterMotor2CanId, MotorType.kBrushed);
        //TODO !! Consider reversing one of the motors for the shooter !!

        encoder1 = motor1.getEncoder();
        //TODO: Setup encoder class to know that its using the clicks from the through bore rev encoder
        // Hardode into the SparkMax config screen and save to SparkMax and check here that it is right and throw error if not


        shooterFeedforward = new SimpleMotorFeedforward(Constants.ShooterkSVolts, Constants.ShooterkWoltSecondsPerRotation);
        getController().setTolerance(Constants.ShooterTolerance);
        setSetpoint(0);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        motor1.setVoltage(output + shooterFeedforward.calculate(setpoint));
        motor2.setVoltage(output + shooterFeedforward.calculate(setpoint));
    }

    @Override
    public double getMeasurement() {
        return encoder1.getVelocity();
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public void stopped()
    {
        setSetpoint(0);
        motor1.stopMotor();
        motor2.stopMotor();
    }



}