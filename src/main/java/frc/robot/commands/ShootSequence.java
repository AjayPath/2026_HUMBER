package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
  public ShootSequence(
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      FloorSubsystem floor,
      DriveSubsystem drive
  ) {
    addCommands(

      // new TurnToAngle(drive, limelight),

      new ParallelCommandGroup(
        // Lock drivetrain in place
        // new RunCommand(() -> drive.setX(), drive),

        new ParallelCommandGroup(
          // Shooter runs the entire time, never interrupted
          new SetShooterRPS(shooter, 65),

          new SequentialCommandGroup(
            // Wait until shooter is up to speed before feeding
            new WaitUntilCommand(() -> shooter.atTargetSpeed()),

            // Then run floor and feeder
            new ParallelCommandGroup(
              new SetFloorRPS(floor, 40),
              new SetFeederRPS(feeder, 90)
            )
          )
        )
      )
    );
  }
}