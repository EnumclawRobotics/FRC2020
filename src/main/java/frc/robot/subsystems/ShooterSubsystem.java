
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;

public class ShooterSubsystem extends PIDSubsystem {
    
    // private CANSparkMax motor1;
    private VictorSPX motor1;
    // private CANSparkMax motor2;
    private VictorSPX motor2;

    // private CANEncoder encoder1;
    private Encoder encoder;
    private SimpleMotorFeedforward shooterFeedforward;

    public ShooterSubsystem() {
        super(new PIDController(Constants.ShooterkP, Constants.ShooterkI, Constants.ShooterkD));
        
        // // redline motors - brushed
        // motor1 = new CANSparkMax(Constants.ShooterMotor1CanId, MotorType.kBrushed);
        motor1 = new VictorSPX(Constants.ShooterMotor1CanId);
        // motor2 = new CANSparkMax(Constants.ShooterMotor2CanId, MotorType.kBrushed);
        motor2 = new VictorSPX(Constants.ShooterMotor2CanId);

        // // through bore encoder
        // encoder1 = new CANEncoder(motor1, EncoderType.kQuadrature, Constants.ShooterEncoderCountsPerRevolution);
        encoder = new Encoder(Constants.ShooterEncoderADIO, Constants.ShooterEncoderBDIO, false, Encoder.EncodingType.k4X);
        encoder.setDistancePerPulse(1/8192);
        shooterFeedforward = new SimpleMotorFeedforward(Constants.ShooterkSVolts, Constants.ShooterkWoltSecondsPerRotation);

        getController().setTolerance(Constants.ShooterTolerance);
        setSetpoint(0);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        double current = output + shooterFeedforward.calculate(setpoint);
        // motor1.setVoltage(output + shooterFeedforward.calculate(setpoint));
        motor1.set(ControlMode.Current, current);
        // motor2.setVoltage(output + shooterFeedforward.calculate(setpoint));
        motor2.set(ControlMode.Current, current);
    }

    @Override
    public double getMeasurement() {
 //       return encoder1.getVelocity();
        return encoder.getRate(); 
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public void stop()
    {
    //     setSetpoint(0);
    //     motor1.stopMotor();
    //     motor2.stopMotor();
 }
}