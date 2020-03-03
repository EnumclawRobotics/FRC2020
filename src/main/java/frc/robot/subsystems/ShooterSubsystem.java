
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class ShooterSubsystem extends PIDSubsystem {
    NetworkTableEntry outputEntry;
    NetworkTableEntry setpointEntry;
    NetworkTableEntry feedforwardEntry;
    NetworkTableEntry getMeasurementEntry;

    // private CANSparkMax motor1;
    private VictorSPX motor1;
    // private CANSparkMax motor2;
    private VictorSPX motor2;

    // private CANEncoder encoder1;
    private Encoder encoder;
    private SimpleMotorFeedforward shooterFeedforward;

    public ShooterSubsystem() {
        super(new PIDController(Constants.ShooterkP, Constants.ShooterkI, Constants.ShooterkD));
        
        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        NetworkTable networkTable = networkTableInstance.getTable("ShooterSubsystem");
        
        feedforwardEntry = networkTable.getEntry("FeedForward");
        getMeasurementEntry = networkTable.getEntry("GetMeasurement");
        outputEntry = networkTable.getEntry("Output");
        setpointEntry = networkTable.getEntry("SetPoint");

        feedforwardEntry.setDouble(0);
        getMeasurementEntry.setDouble(0);
        outputEntry.setDouble(0);
        setpointEntry.setDouble(0);

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
        setSetpoint(Constants.ShooterFreeThrowRPS); //5000/60 
    }

    @Override
    public void useOutput(double output, double setpoint) {
        double feedForward = shooterFeedforward.calculate(setpoint);
        double current = output + feedForward;
        double percent = output;

        motor1.set(ControlMode.PercentOutput, output);
        motor2.set(ControlMode.PercentOutput, output);

        //motor1.setVoltage(output + shooterFeedforward.calculate(setpoint));
        //motor2.setVoltage(output + shooterFeedforward.calculate(setpoint));

        feedforwardEntry.setDouble(feedForward);
        getMeasurementEntry.setDouble(getMeasurement());
        outputEntry.setDouble(output);
        setpointEntry.setDouble(setpoint);
    }

    @Override
    public double getMeasurement() {
        return encoder.getRate(); 
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public void stop()
    {
        this.disable();
        motor1.set(ControlMode.Current, 0);
        motor2.set(ControlMode.Current, 0);
    }
}