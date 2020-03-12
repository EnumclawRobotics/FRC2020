
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class ShooterSubsystem extends PIDSubsystem {
    NetworkTableEntry atSetpointEntry;
    NetworkTableEntry feedforwardEntry;
    NetworkTableEntry getMeasurementEntry;
    NetworkTableEntry outputEntry;
    NetworkTableEntry percentEntry;
    NetworkTableEntry setpointEntry;

    private VictorSPX motor1;
    private VictorSPX motor2;

    private DutyCycleEncoder encoder;

    // ramp
    private double previousPercent = 0;

    // to calc frequency
    private double previousDistance = 0;
    private long previousTime = 0;
    private double frequency = 0;

    private double targetRPS = 0;
    private double feedForward = 0;

    public ShooterSubsystem() {
        super(new PIDController(Constants.ShooterkP, Constants.ShooterkI, Constants.ShooterkD));
        
        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        NetworkTable networkTable = networkTableInstance.getTable("ShooterSubsystem");
        
        atSetpointEntry = networkTable.getEntry("AtSetPoint");
        feedforwardEntry = networkTable.getEntry("FeedForward Constant");
        getMeasurementEntry = networkTable.getEntry("GetMeasurement");
        outputEntry = networkTable.getEntry("Output PID");
        percentEntry = networkTable.getEntry("Percent");
        setpointEntry = networkTable.getEntry("SetPoint");

        atSetpointEntry.setBoolean(false);
        feedforwardEntry.setDouble(0);
        getMeasurementEntry.setDouble(0);
        outputEntry.setDouble(0);
        percentEntry.setDouble(0);
        setpointEntry.setDouble(0);

        // // redline motors - brushed
        motor1 = new VictorSPX(Constants.ShooterMotor1CanId);
        motor2 = new VictorSPX(Constants.ShooterMotor2CanId);

        // through bore encoder
        encoder = new DutyCycleEncoder(3);    // using absolute white wire 1rev per pulse
    
        getController().setTolerance(Constants.ShooterToleranceRPS);
        getController().setSetpoint(0);

        this.disable();
    }

    public void shoot() {
        this.targetRPS = Constants.ShooterFreeThrowRPS;
        this.setSetpoint(this.targetRPS);
        this.feedForward = Constants.ShooterkFF;
    }

    public void intake() {
        this.targetRPS = Constants.ShooterIntakeRPS;
        this.setSetpoint(this.targetRPS);
        this.feedForward = Constants.ShooterkFFIntake;
    }

    @Override
    public void useOutput(double output, double setpoint) {
        double percent = 0;

        if (this.isEnabled()) {
            percent = this.feedForward + output;

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
        } 

        // set motors to run
        motor1.set(ControlMode.PercentOutput, percent);
        motor2.set(ControlMode.PercentOutput, percent);

        atSetpointEntry.setBoolean(this.atSetpoint());
        feedforwardEntry.setDouble(this.feedForward);
        getMeasurementEntry.setDouble(this.getMeasurement());
        outputEntry.setDouble(output);
        percentEntry.setDouble(percent);
        setpointEntry.setDouble(setpoint);
    }

    @Override
    public double getMeasurement() {
        return getFrequency(); 
    }

    public void updateFrequency() {
        double distance =  getDistance();
        long time = System.currentTimeMillis();

        // do we have values? calc
        if (previousTime != 0 || previousDistance != 0) {
            this.frequency = ((distance - previousDistance) / ( time - previousTime)) * 1000;
        }

        // update 
        previousTime = time;
        previousDistance = distance;
    }

    // distance per sec
    public double getFrequency() {
        return this.frequency;
    }

    public double getDistance() {
        return encoder.getDistance(); 
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public void stop()
    {
        // this.disable();
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
        this.previousPercent = 0;
    }
}