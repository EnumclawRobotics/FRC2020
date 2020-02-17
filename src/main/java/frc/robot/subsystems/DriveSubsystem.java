package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final SpeedControllerGroup leftMotors =
      new SpeedControllerGroup(new CANSparkMax(Constants.FrontLeftMotorCanId, MotorType.kBrushless),
                               new CANSparkMax(Constants.BackLeftMotorCanId, MotorType.kBrushless));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup rightMotors =
      new SpeedControllerGroup(new CANSparkMax(Constants.FrontRightMotorCanId, MotorType.kBrushless),
                               new CANSparkMax(Constants.BackRightMotorCanId, MotorType.kBrushless));

  // The robot's drive
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  private Encoder leftEncoder = new Encoder(000, 000, false); //TODO Ports for drive train encoders
  private Encoder rightEncoder = new Encoder(000, 000, true);

  //Gyro
  private final Gyro gyro = new ADXRS450_Gyro();

  //Webpage of interest for gyro https://wiki.analog.com/first/adxrs450_gyro_board_frc

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param forward 
   * @param rotation
   */
  public void arcadeDrive(double forward, double rotation) {
    drive.arcadeDrive(forward, rotation);
  }

  public double getEncoderDistance()
  {
    return leftEncoder.getDistance(); //TODO Factor in setDistancePerPulse(), Motor Encoder units per revolution, size of wheel
  }

  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.GyroReversed ? -1.0 : 1.0);
  }

}