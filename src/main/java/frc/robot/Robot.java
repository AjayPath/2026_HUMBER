// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.DriveSubsystem;

// /**
//  * The methods in this class are called automatically corresponding to each mode, as described in
//  * the TimedRobot documentation. If you change the name of this class or the package after creating
//  * this project, you must also update the Main.java file in the project.
//  */
// public class Robot extends TimedRobot {
//   private Command m_autonomousCommand;

//   private final RobotContainer m_robotContainer;

//   /**
//    * This function is run when the robot is first started up and should be used for any
//    * initialization code.
//    */
//   public Robot() {
//     // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
//     // autonomous chooser on the dashboard.
//     m_robotContainer = new RobotContainer();
//     initializeGyro();
//   }

//   /**
//    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
//    * that you want ran during disabled, autonomous, teleoperated and test.
//    *
//    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
//    * SmartDashboard integrated updating.
//    */
//   @Override
//   public void robotPeriodic() {
//     // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
//     // commands, running already-scheduled commands, removing finished or interrupted commands,
//     // and running subsystem periodic() methods.  This must be called from the robot's periodic
//     // block in order for anything in the Command-based framework to work.
//     CommandScheduler.getInstance().run();
//   }

//   /** This function is called once each time the robot enters Disabled mode. */
//   @Override
//   public void disabledInit() {}

//   @Override
//   public void disabledPeriodic() {}

//   /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//   @Override
//   public void autonomousInit() {
//     m_autonomousCommand = m_robotContainer.getAutonomousCommand();

//     // schedule the autonomous command (example)
//     if (m_autonomousCommand != null) {
//       CommandScheduler.getInstance().schedule(m_autonomousCommand);
//     }
//   }

//   /** This function is called periodically during autonomous. */
//   @Override
//   public void autonomousPeriodic() {}

//   @Override
//   public void teleopInit() {
//     // This makes sure that the autonomous stops running when
//     // teleop starts running. If you want the autonomous to
//     // continue until interrupted by another command, remove
//     // this line or comment it out.
//     if (m_autonomousCommand != null) {
//       m_autonomousCommand.cancel();
//     }
//   }

//   /** This function is called periodically during operator control. */
//   @Override
//   public void teleopPeriodic() {}

//   @Override
//   public void testInit() {
//     // Cancels all running commands at the start of test mode.
//     CommandScheduler.getInstance().cancelAll();
//   }

//   /** This function is called periodically during test mode. */
//   @Override
//   public void testPeriodic() {}

//   /** This function is called once when the robot is first started up. */
//   @Override
//   public void simulationInit() {}

//   /** This function is called periodically whilst in simulation. */
//   @Override
//   public void simulationPeriodic() {}

//   private void initializeGyro() {
//     DriveSubsystem drive = m_robotContainer.getDriveSubsystem();
    
//     new Thread(() -> {
//         try {
//             // Wait for gyro to stabilize after power-on
//             Thread.sleep(1000);
//         } catch (InterruptedException e) {
//             e.printStackTrace();
//         }
        
//         // Reset gyro to zero (includes wait for valid readings)
//         drive.resetGyroToZero();
//     }).start();
//   }
// }


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
  private static final String kAutoLeft   = "Left Side Auto";
  private static final String kAutoMiddle = "Middle Auto";
  private static final String kAutoRight  = "Right Side Auto";

  public Robot() {
    m_robotContainer = new RobotContainer();

    m_autoChooser.setDefaultOption("Left Side Auto", kAutoLeft);
    m_autoChooser.addOption("Middle Auto", kAutoMiddle);
    m_autoChooser.addOption("Right Side Auto", kAutoRight);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    initializeGyro();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    String autoSelected = m_autoChooser.getSelected();
    SmartDashboard.putString("Auto Running", autoSelected);

    switch (autoSelected) {
      case kAutoLeft:
        m_autonomousCommand = m_robotContainer.getLeftSideAuto();
        break;
      case kAutoRight:
        m_autonomousCommand = m_robotContainer.getRightSideAuto();
        break;
      case kAutoMiddle:
      default:
        m_autonomousCommand = m_robotContainer.getMiddleAuto();
        break;
    }

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void initializeGyro() {
    DriveSubsystem drive = m_robotContainer.getDriveSubsystem();

    new Thread(() -> {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      drive.resetGyroToZero();
    }).start();
  }
}