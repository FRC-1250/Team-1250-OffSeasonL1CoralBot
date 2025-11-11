// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Arm.Positions;
import frc.robot.Commands.ArmMovement;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }
  public final Arm arm = new Arm();
  private void configureBindings() {
    new ArmMovement(arm, Positions.Home);
    new ArmMovement(arm, Positions.Climb);
    new ArmMovement(arm, Positions.Intake);
    new ArmMovement(arm, Positions.ScoreCoral);
    new ArmMovement(arm, Positions.ClimbPrep);
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
