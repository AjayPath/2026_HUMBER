// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Purge extends Command {
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final FloorSubsystem floorSubsystem;

  /** Creates a new Purge. */
  public Purge(FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, FloorSubsystem floorSubsystem) {
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.floorSubsystem = floorSubsystem;
    
    addRequirements(feederSubsystem, shooterSubsystem, intakeSubsystem, floorSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Variables.feeder.feederRPS = -80;
    Variables.shooter.shooterRPS = -80;
    Variables.floor.floorRPS = -80;
    Variables.intake.intakeRPS = -80;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Variables.feeder.feederRPS = 0;
    Variables.shooter.shooterRPS = 0;
    Variables.floor.floorRPS = 0;
    Variables.intake.intakeRPS = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}