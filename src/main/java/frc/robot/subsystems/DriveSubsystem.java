package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

}