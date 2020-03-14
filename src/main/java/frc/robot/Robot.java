/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootingTimedCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeHopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/* TODO Overall Todo List
    - Control Panel (Low Priority)
      - Add Flipout motor
      - Rotation Control Function
      - Get color for Position Control
      - Position Control Function

    - Autonomous (Med/High Priority)
      - Drive encoders
      - Get Gyro Data
      - Driving functions

    - Physical Related (High Priority)
      - ID the motors and update constants accordingly (Did SparkMax Motors)
      - Specify other data such as RPM and Brushed/Brushless (Did SparkMax Motors)
      - Switch needed Motors to be operated with VictorSPX (Done until more need replacement)

    - To keep note of, listen for team updates on
      - Hook lock to stop sliding end of match
      - Where control panel color sensor will be


*/

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private DriveSubsystem driveSubsystem;
  //private IntakeHopperShooterSubsystem intakeHopperShooterSubsystem;
  private IntakeHopperSubsystem intakeHopperSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ArmSubsystem armSubsystem;
  private WinchSubsystem winchSubsystem;
//  private HookSubsystem hookSubsystem;
  private XboxController xboxController1;
  private XboxController xboxController2;
  
  //private Command m_autonomousCommand;
  private Command autonomousCommand;

  private NetworkTableEntry shooterEntry;

  private static boolean driveNormal = true;

  //private final Timer shooterTimer = new Timer();
  private boolean shooterActivated = false;

  private static boolean slowedDown = false;

  //private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
//    m_robotContainer = new RobotContainer();
  
    xboxController1 = new XboxController(0);
    xboxController2 = new XboxController(1);
    driveSubsystem = new DriveSubsystem();
    intakeHopperSubsystem = new IntakeHopperSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    armSubsystem = new ArmSubsystem();
    winchSubsystem = new WinchSubsystem();
    //hookSubsystem = new HookSubsystem();
  
//    usbCamera0 = CameraServer.getInstance().startAutomaticCapture(0);
//    usbCamera1 = CameraServer.getInstance().startAutomaticCapture(1);

    startCaptureWithCenterLine(0, "Intake Camera");
    startCaptureWithCenterLine(1, "Shooter Camera");

    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    NetworkTable networkTable = networkTableInstance.getTable("ShooterSubsystem");
    
    shooterEntry = networkTable.getEntry("Test Entry");
  }

  public static boolean isDriveNormal() {
    return driveNormal;
  }

  public static boolean isSlowedDown() {
    return slowedDown;
  }

  private void startCaptureWithCenterLine(int usbId, String cameraName) {
    new Thread(() -> {
      CameraServer.getInstance().startAutomaticCapture(cameraName, usbId);
     //camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getInstance().getVideo(cameraName);
      CvSource outputStream = CameraServer.getInstance().putVideo(cameraName, 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      Scalar blackColor = new Scalar(0,0,0);
      Scalar greenColor = new Scalar(0,255,0);
      Scalar yellowColor = new Scalar(0,255,255);   // BGR 

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }

        // draw center line
        Imgproc.line(source, new Point(source.width()/2, 0), 
          new Point(source.width()/2, source.height()), yellowColor, 2);

        // intake should be 0, is that front?
        if ((Robot.isDriveNormal() && usbId == 0) 
            || (!Robot.isDriveNormal() && usbId == 1)) {

          // build text
          String showText = "FRONT";
          Imgproc.putText(source, showText, new Point(source.width()*.15, 30), 0, .75, blackColor, 3);
          Imgproc.putText(source, showText, new Point(source.width()*.15, 30), 0, .75, yellowColor, 2);

          if (Robot.isSlowedDown()) {
            showText = "SLOWED";
            Imgproc.putText(source, showText, new Point(source.width()*.55, 30), 0, .75, blackColor, 3);
            Imgproc.putText(source, showText, new Point(source.width()*.55, 30), 0, .75, yellowColor, 2);
          }
        }

        outputStream.putFrame(source);
      }

    }).start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    shooterSubsystem.updateFrequency();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
      this.shooterSubsystem.disable();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

    autonomousCommand = new SequentialCommandGroup(new ShootingTimedCommand(10, Constants.ShooterFreeThrowRPS, intakeHopperSubsystem, shooterSubsystem));
    if (autonomousCommand != null)
    {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
    if (autonomousCommand != null)
    {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    shooterEntry.setDouble(shooterSubsystem.getFrequency());

    //Slow Down Driving? (Toggle with X button)
    if (xboxController1.getXButtonPressed())
    {
      slowedDown = !slowedDown;
    }

    //Driving, Left Bumper reverses forward/backward
    if (xboxController1.getBumperPressed(Hand.kLeft))
    {
      driveNormal = !driveNormal;
    }

    double forwardSpeed = xboxController1.getY(Hand.kLeft);
    double turningSpeed = -xboxController1.getX(Hand.kRight);
    if (!driveNormal)
    {
      forwardSpeed *= -1;
    }
    if (slowedDown)
    {
      forwardSpeed *= Constants.DriveSlowDownForward;
      turningSpeed *= Constants.DriveSlowDownTurning;
    }
    driveSubsystem.arcadeDrive(forwardSpeed, turningSpeed);

        // Winch (2nd controller - left joy y)
    winchSubsystem.setPower(-xboxController2.getY(Hand.kLeft));

    //Arm (2nd controller - right joy y )
    armSubsystem.setPower(xboxController2.getY(Hand.kRight));
 

    // Hook - rolling on bar (15 is dpad right, 14 is dpad left) (dpad Right - dpad Left) 
    //TODO Check if this method of getting dpad input is correct or not. Controller should be a wired Xbox 360 controller. If using getPOV, which POV (int) is it?
//    hookSubsystem.traversePower(Constants.HookTraversePower * ((xboxController.getRawButton(15) ? 1 : 0) - (xboxController.getRawButton(14) ? 1 : 0)));

    // SHOOT
    if (xboxController1.getTriggerAxis(Hand.kLeft) > 0.25) {
      if (!shooterActivated)
      {
        //shooterTimer.reset();
        //shooterTimer.start();
        shooterActivated = true;

        shooterSubsystem.shoot();
        shooterSubsystem.enable();

        intakeHopperSubsystem.chargeIntakeForShooter();
      }

      if (shooterSubsystem.atSetpoint())
      {
        intakeHopperSubsystem.transportToShooter();
      }
    }
    else {
      shooterActivated = false;

      // INTAKE
      if (xboxController1.getBumper(Hand.kRight)) {
        shooterSubsystem.intake();
        shooterSubsystem.enable();
        intakeHopperSubsystem.intake();
      }
      // EXPEL
      else if (xboxController1.getTriggerAxis(Hand.kRight) > 0.25) {
        shooterSubsystem.disable();
        shooterSubsystem.stop();
        intakeHopperSubsystem.expel();
      }
      // STOP
      else {
        // TODO: in shooter subsytem dont all disable if already disabled
        shooterSubsystem.disable();
        shooterSubsystem.stop();
        intakeHopperSubsystem.stop();
      }
    }

    // panel flipout and rotator
    // [Put code here]
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    shooterEntry.setDouble(shooterSubsystem.getFrequency());
  }
}
