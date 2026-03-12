package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.Pose;

public class ResetPose extends Command {
  /** Creates a new ResetPose. */
  private final DriveSubsystem driveSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  public ResetPose(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(driveSubsystem);
    addRequirements(limelightSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose visionPose = limelightSubsystem.getPoseFromTag(
        driveSubsystem.getPoseContinuous().getAngle()
    );

    if (visionPose != null) {
        driveSubsystem.resetOdometry(visionPose);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}