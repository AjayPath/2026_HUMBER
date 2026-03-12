// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Variables;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.utils.APPID;
// import frc.robot.utils.Calculations;

// public class TurnToTagSnapshot extends Command {
//   private final DriveSubsystem driveSubsystem;
//   private final APPID turnPID;

//   private double targetAngleDeg;

//   private static final double kTurnP = 0.022;
//   private static final double kTurnI = 0.0;
//   private static final double kTurnD = 0.0;
//   private static final double kMaxRot = 0.30;
//   private static final double kMinRot = 0.06;

//   private static final double kAngleDeadbandDeg = 1.25;
//   private static final double kMinRotActiveDeg = 3.0;

//   public TurnToTagSnapshot(DriveSubsystem driveSubsystem) {
//     this.driveSubsystem = driveSubsystem;
//     this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, 0.0);
//     this.turnPID.setMaxOutput(kMaxRot);

//     addRequirements(driveSubsystem);
//   }

//   @Override
//   public void initialize() {
//     turnPID.reset();

//     double currentAngle = Calculations.normalizeAngle360(driveSubsystem.getPose().getAngle());

//     if (Variables.limelight.hasValidTarget) {
//       double capturedTx = Variables.limelight.tX;
//       targetAngleDeg = Calculations.normalizeAngle360(currentAngle - capturedTx);
//     } else {
//       targetAngleDeg = currentAngle;
//     }
//   }

//   @Override
//   public void execute() {
//     double currentAngle = Calculations.normalizeAngle360(driveSubsystem.getPose().getAngle());
//     double angleError = Calculations.shortestAngularDistance(targetAngleDeg, currentAngle);

//     double rotCmd = 0.0;

//     if (Math.abs(angleError) > kAngleDeadbandDeg) {
//       turnPID.setDesiredValue(0.0);
//       rotCmd = turnPID.calculate(-angleError);

//       if (Math.abs(angleError) > kMinRotActiveDeg && Math.abs(rotCmd) < kMinRot) {
//         rotCmd = Math.copySign(kMinRot, -angleError);
//       }
//     }

//     driveSubsystem.drive(0.0, 0.0, rotCmd, true);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     driveSubsystem.drive(0.0, 0.0, 0.0, true);
//   }

//   @Override
//   public boolean isFinished() {
//     double currentAngle = Calculations.normalizeAngle360(driveSubsystem.getPose().getAngle());
//     double angleError = Math.abs(Calculations.shortestAngularDistance(targetAngleDeg, currentAngle));
//     return angleError <= kAngleDeadbandDeg;
//   }
// }package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Variables;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.utils.APPID;
// import frc.robot.utils.Calculations;

// public class TurnToTagSnapshot extends Command {
//   private final DriveSubsystem driveSubsystem;
//   private final APPID turnPID;

//   private double targetAngleDeg;

//   private static final double kTurnP = 0.022;
//   private static final double kTurnI = 0.0;
//   private static final double kTurnD = 0.0;
//   private static final double kMaxRot = 0.30;
//   private static final double kMinRot = 0.06;

//   private static final double kAngleDeadbandDeg = 1.25;
//   private static final double kMinRotActiveDeg = 3.0;

//   public TurnToTagSnapshot(DriveSubsystem driveSubsystem) {
//     this.driveSubsystem = driveSubsystem;
//     this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, 0.0);
//     this.turnPID.setMaxOutput(kMaxRot);

//     addRequirements(driveSubsystem);
//   }

//   @Override
//   public void initialize() {
//     turnPID.reset();

//     double currentAngle = Calculations.normalizeAngle360(driveSubsystem.getPose().getAngle());

//     if (Variables.limelight.hasValidTarget) {
//       double capturedTx = Variables.limelight.tX;
//       targetAngleDeg = Calculations.normalizeAngle360(currentAngle - capturedTx);
//     } else {
//       targetAngleDeg = currentAngle;
//     }
//   }

//   @Override
//   public void execute() {
//     double currentAngle = Calculations.normalizeAngle360(driveSubsystem.getPose().getAngle());
//     double angleError = Calculations.shortestAngularDistance(targetAngleDeg, currentAngle);

//     double rotCmd = 0.0;

//     if (Math.abs(angleError) > kAngleDeadbandDeg) {
//       turnPID.setDesiredValue(0.0);
//       rotCmd = turnPID.calculate(-angleError);

//       if (Math.abs(angleError) > kMinRotActiveDeg && Math.abs(rotCmd) < kMinRot) {
//         rotCmd = Math.copySign(kMinRot, -angleError);
//       }
//     }

//     driveSubsystem.drive(0.0, 0.0, rotCmd, true);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     driveSubsystem.drive(0.0, 0.0, 0.0, true);
//   }

//   @Override
//   public boolean isFinished() {
//     double currentAngle = Calculations.normalizeAngle360(driveSubsystem.getPose().getAngle());
//     double angleError = Math.abs(Calculations.shortestAngularDistance(targetAngleDeg, currentAngle));
//     return angleError <= kAngleDeadbandDeg;
//   }
// }

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;

public class TurnToTagLive extends Command {

  private final DriveSubsystem driveSubsystem;
  private final APPID turnPID;

  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.0;

  private static final double kMaxRot = 0.30;
  private static final double kMinRot = 0.05;
  private static final double kToleranceDeg = 1;

  public TurnToTagLive(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    turnPID = new APPID(kTurnP, kTurnI, kTurnD, 0.0);
    turnPID.setMaxOutput(kMaxRot);

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    turnPID.reset();
  }

  @Override
  public void execute() {

    if (!Variables.limelight.hasValidTarget) {
      driveSubsystem.drive(0.0, 0.0, 0.0, true);
      return;
    }

    double tx = Variables.limelight.tX;

    turnPID.setDesiredValue(0.0);
    double rot = -turnPID.calculate(-tx);

    if (Math.abs(tx) > 2.0 && Math.abs(rot) < kMinRot) {
      rot = Math.copySign(kMinRot, -tx);
    }

    driveSubsystem.drive(0.0, 0.0, rot, true);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return Variables.limelight.hasValidTarget &&
           Math.abs(Variables.limelight.tX) <= kToleranceDeg;
  }
}