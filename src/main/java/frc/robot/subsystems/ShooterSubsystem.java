
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class ShooterSubsystem extends PIDSubsystem {
    NetworkTableEntry feedforwardEntry;
    NetworkTableEntry getMeasurementEntry;
    NetworkTableEntry outputEntry;
    NetworkTableEntry percentEntry;
    NetworkTableEntry setpointEntry;

    // private CANSparkMax motor1;
    private VictorSPX motor1;
    // private CANSparkMax motor2;
    private VictorSPX motor2;

    private DutyCycleEncoder encoder;
    // private SimpleMotorFeedforward shooterFeedforward;
    private double previousPercent = 0;

    private double targetShootingPower = 0;

    public ShooterSubsystem() {
        super(new PIDController(Constants.ShooterkP, Constants.ShooterkI, Constants.ShooterkD));
        
        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        NetworkTable networkTable = networkTableInstance.getTable("ShooterSubsystem");
        
        feedforwardEntry = networkTable.getEntry("FeedForward Constant");
        getMeasurementEntry = networkTable.getEntry("GetMeasurement");
        outputEntry = networkTable.getEntry("Output PID");
        percentEntry = networkTable.getEntry("Percent");
        setpointEntry = networkTable.getEntry("SetPoint");

        feedforwardEntry.setDouble(0);
        getMeasurementEntry.setDouble(0);
        outputEntry.setDouble(0);
        percentEntry.setDouble(0);
        setpointEntry.setDouble(0);

        // // redline motors - brushed
        // motor1 = new CANSparkMax(Constants.ShooterMotor1CanId, MotorType.kBrushed);
        motor1 = new VictorSPX(Constants.ShooterMotor1CanId);
        // motor2 = new CANSparkMax(Constants.ShooterMotor2CanId, MotorType.kBrushed);
        motor2 = new VictorSPX(Constants.ShooterMotor2CanId);

        // // through bore encoder
        // encoder1 = new CANEncoder(motor1, EncoderType.kQuadrature, Constants.ShooterEncoderCountsPerRevolution);
        encoder = new DutyCycleEncoder(3);
//        encoder.setDistancePerPulse(1/8192);
        // shooterFeedforward = new SimpleMotorFeedforward(Constants.ShooterkSVolts, Constants.ShooterkWoltSecondsPerRotation);

        getController().setTolerance(Constants.ShooterToleranceRPS);
        getController().setSetpoint(Constants.ShooterFreeThrowRPS); //5000/60 
    }

    public void setTargetPower(double power)
    {
        targetShootingPower = power;
    }

    @Override
    public void useOutput(double output, double setpoint) {
        //double feedForward = shooterFeedforward.calculate(setpoint);
        //double current = output + feedForward;
        //double percent = output + Constants.ShooterkFF;
        double percent = targetShootingPower;

        // limit change by ramp
        if (percent - this.previousPercent > Constants.ShooterRamp) {
            percent = this.previousPercent + Constants.ShooterRamp;
        } else if (this.previousPercent - percent > Constants.ShooterRamp) {
            percent = this.previousPercent - Constants.ShooterRamp;
        }

        // limit by absolute bounds
        if (percent > Constants.ShooterkMaxOutput) {
            percent = Constants.ShooterkMaxOutput;
        } else if (percent < Constants.ShooterkMinOutput) {
            percent = Constants.ShooterkMinOutput;
        }

        // update previousPercent
        this.previousPercent = percent;

        // set motors to run
        motor1.set(ControlMode.PercentOutput, percent);
        motor2.set(ControlMode.PercentOutput, percent);

        //motor1.setVoltage(current);
        //motor2.setVoltage(current);

        //feedforwardEntry.setDouble(feedForward);
        feedforwardEntry.setDouble(Constants.ShooterkFF);
        getMeasurementEntry.setDouble(getMeasurement());
        outputEntry.setDouble(output);
        percentEntry.setDouble(percent);
        setpointEntry.setDouble(setpoint);
    }

    @Override
    public double getMeasurement() {
        return 0;
//        return encoder.getFrequency(); 
    }

    public double getDistance() {
        return encoder.getDistance(); 
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public void stop()
    {
        this.disable();
        motor1.set(ControlMode.Current, 0);
        motor2.set(ControlMode.Current, 0);
        this.previousPercent = 0;
    }
}