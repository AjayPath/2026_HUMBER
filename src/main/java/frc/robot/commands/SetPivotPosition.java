package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.PivotSubsystem;

public class SetPivotPosition extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final double position;
  
  public final double TOLERANCE_DEGREES = 2.0;

  public SetPivotPosition(PivotSubsystem pivotSubsystem, double position) {
    this.pivotSubsystem = pivotSubsystem;
    this.position = position;

    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {
    Variables.pivot.pivotPosition = position;
  }

  @Override
  public void execute() {

  }


  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return pivotSubsystem.pivotAtTarget(position, TOLERANCE_DEGREES);
  }
}