// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Purge;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.TurnToTagLive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

        // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
      .whileTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
      .whileTrue(new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, Variables.limelight.shooterRPS));

    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
      .whileTrue(new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 30, 110));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whileTrue(new Purge(s_feederSubsystem, s_shooterSubsystem, s_intakeSubsystem, s_floorSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .whileTrue(new SetPivotPosition(s_pivotSubsystem, 10));
    
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, 65));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new TurnToTagLive(m_robotDrive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }
}
