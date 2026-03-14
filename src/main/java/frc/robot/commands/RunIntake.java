// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;

  private final double rps;
  private final double position;

  public RunIntake(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, double rps, double position) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;

    this.rps = rps;
    this.position = position;

    addRequirements(intakeSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Variables.pivot.pivotPosition = position;
  }

  @Override
  public void execute() {
    Variables.intake.intakeRPS = rps;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Variables.intake.intakeRPS = 0;
    Variables.pivot.pivotPosition = 119;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}