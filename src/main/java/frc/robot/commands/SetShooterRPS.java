// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShooterRPS extends Command {
  private final ShooterSubsystem ShooterSubsystem;
  private final double rps;
  private boolean isLimelight;
  
  public SetShooterRPS(ShooterSubsystem ShooterSubsystem, double rps) {
    this.ShooterSubsystem = ShooterSubsystem;
    this.rps = rps;
    this.isLimelight = false;

    addRequirements(ShooterSubsystem);
  }

  public SetShooterRPS(ShooterSubsystem ShooterSubsystem) {
    this.ShooterSubsystem = ShooterSubsystem;
    this.rps = 0;
    this.isLimelight = true;

    addRequirements(ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isLimelight) {
      Variables.shooter.shooterRPS = Variables.limelight.shooterRPS;
    } else {
      Variables.shooter.shooterRPS = rps;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Variables.shooter.shooterRPS = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}