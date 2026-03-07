// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.FloorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetFloorRPS extends Command {
  private final FloorSubsystem floorSubsystem;
  private final double rps;
  
  public SetFloorRPS(FloorSubsystem floorSubsystem, double rps) {
    this.floorSubsystem = floorSubsystem;
    this.rps = rps;

    addRequirements(floorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Variables.floor.floorRPS = rps;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Variables.floor.floorRPS = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}