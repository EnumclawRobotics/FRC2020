/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class ControlPanelSubsystem extends SubsystemBase {
  private CANSparkMax panelMotor = new CANSparkMax(Constants.PanelMotorCanId, MotorType.kBrushless);
  private ColorSensorV3 colorSensor = new ColorSensorV3(Constants.ColorSensorPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */

  /**
   * Creates a new ExampleSubsystem.
   */
  public ControlPanelSubsystem() {
    m_colorMatcher.addColorMatch(Constants.kBlueTarget);
    m_colorMatcher.addColorMatch(Constants.kGreenTarget);
    m_colorMatcher.addColorMatch(Constants.kRedTarget);
    m_colorMatcher.addColorMatch(Constants.kYellowTarget);
  }

  public Color getColor()
  {
    return colorSensor.getColor();
  }


  public String getColorMatch()
  {
    Color color = getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(color);

    if (match.color == Constants.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == Constants.kRedTarget) {
      colorString = "Red";
    } else if (match.color == Constants.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == Constants.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    return colorString;
  }

  public void spinPanel(double power)
  {
    panelMotor.set(power * Constants.MaxPanelMotorPower);
  }

  public void stopped()
  {
    panelMotor.stopMotor();
  }
}
