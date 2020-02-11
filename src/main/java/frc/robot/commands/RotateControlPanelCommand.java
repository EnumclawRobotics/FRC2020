/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * An example command that uses an example subsystem.
 */
public class RotateControlPanelCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ControlPanelSubsystem subsystem;
    private int colorSwitches;
    String colorMatch;
    String previousColorMatch;
    private final ColorMatch colorMatcher = new ColorMatch();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
//  public ExampleCommand(ExampleSubsystem subsystem) {
//    m_subsystem = subsystem;
//    // Use addRequirements() here to declare subsystem dependencies.
//    addRequirements(subsystem);
//  }

    /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateControlPanelCommand(int colorSwitches, ControlPanelSubsystem subsystem) {
      this.colorSwitches = colorSwitches;
      this.subsystem = subsystem;
      addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorMatch = subsystem.getColorMatch();
    previousColorMatch = colorMatch;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      colorMatch = subsystem.getColorMatch();
      if (colorMatch != previousColorMatch)
      {
          colorSwitches--;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
