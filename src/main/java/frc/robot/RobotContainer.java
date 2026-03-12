// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.PassSequence;
import frc.robot.commands.Purge;
import frc.robot.commands.ResetPose;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToTagLive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private final FloorSubsystem s_floorSubsystem = new FloorSubsystem();
  private final FeederSubsystem s_feederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem s_intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem s_pivotSubsystem = new PivotSubsystem();
  private final LimelightSubsystem s_limelightSubsystem = new LimelightSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
      .whileTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
      .whileTrue(new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem));

    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
      .whileTrue(new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 80, 119));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(new Purge(s_feederSubsystem, s_shooterSubsystem, s_intakeSubsystem, s_floorSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .whileTrue(new SetPivotPosition(s_pivotSubsystem, 25));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whileTrue(new PassSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem, 80));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new TurnToTagLive(m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
      .whileTrue(new ResetPose(m_robotDrive, s_limelightSubsystem));

    new Trigger(() -> s_limelightSubsystem.isRumbleTagVisible())
    .onTrue(new InstantCommand(
        () -> m_driverController.setRumble(RumbleType.kBothRumble, 1.0)))
    .onFalse(new InstantCommand(
        () -> m_driverController.setRumble(RumbleType.kBothRumble, 0.0)));
   }

  public Command getLeftSideAuto() {

    return 
    new SequentialCommandGroup(
          new ParallelCommandGroup(
          new SetPivotPosition(s_pivotSubsystem, 119),
          new ResetPose(m_robotDrive, s_limelightSubsystem)
          ),

          new DriveToPoint(m_robotDrive, 2.8, 0, 0, 0.25, 5),
          new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, 3.7, -1, 270, 0.25, 5),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 80, 119)
          ),

          new ParallelDeadlineGroup(
            new DriveToPoint(m_robotDrive, 3.7, -4, 270, 0.25, 5),
            new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 80, 119)
          ),

            new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, 2.5, -0.35, 180, 0.5, 5),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 80, 119)
           ),

           new WaitCommand(0.2),
           new ResetPose(m_robotDrive, s_limelightSubsystem),
           new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, -2, 0, 180, 0.25, 3),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 80, 119)
           ),
          new TurnToAngle(m_robotDrive, 310),
          new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem).withTimeout(4),
          new TurnToAngle(m_robotDrive, 0),
          new ResetPose(m_robotDrive, s_limelightSubsystem),
          new DriveToPoint(m_robotDrive, 2.3, 0, 0, 0.25, 5),
          new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, 2.3, -4, 270, 0.25, 5),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 80, 119)
           ),
          new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, 2.3, -0.35, 180, 0.25, 5),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 80, 119)
           ),
          new WaitCommand(0.25),
          new ResetPose(m_robotDrive, s_limelightSubsystem),
            new ParallelDeadlineGroup(
            new DriveToPoint(m_robotDrive, -2.2, 0, 180, 0.25, 3),
            new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 80, 119)
          ),
          new TurnToAngle(m_robotDrive, 310),
          new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem).withTimeout(4)
          ); // add your left side auto commands here
  }

  public Command getMiddleAuto() {
    return 
    new SequentialCommandGroup(
      new TurnToAngle(m_robotDrive, 300),
      new SetPivotPosition(s_pivotSubsystem, 80),
      new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem).withTimeout(2),
      new TurnToAngle(m_robotDrive, 0),
      new ParallelCommandGroup(
           new SetPivotPosition(s_pivotSubsystem, 119),
           new ResetPose(m_robotDrive, s_limelightSubsystem)
      ),
      new DriveToPoint(m_robotDrive, 2.8, 0, 0, 0.5, 5),
           new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, 3.7, -1, 270, 0.25, 5),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 40, 119)
           ),
           new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, 3.7, -4, 270, 0.25, 5),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 40, 119)
           ),
            new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, 2.5, -0.35, 180, 0.5, 5),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 40, 119)
           ),
           new WaitCommand(0.2),
           new ResetPose(m_robotDrive, s_limelightSubsystem),
          new ParallelDeadlineGroup(
             new DriveToPoint(m_robotDrive, -1.8, 0, 180, 0.25, 3),
             new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 40, 119)
           ),
            new TurnToAngle(m_robotDrive, 310),
            new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem).withTimeout(4)

    ); // add your middle auto commands here
  }

  public Command getRightSideAuto() {
    return null; // add your right side auto commands here
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }
}