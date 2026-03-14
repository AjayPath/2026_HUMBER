package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.utils.APTree;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

public class LimelightSubsystem extends SubsystemBase {

  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  private static APTree tagToDistanceLookup = new APTree();
  private static APTree distanceToSpeedLookup = new APTree();
  
  private static final double CAMERA_TO_CENTRE = 0.349;

  // -------------------------------------------------------
  //   Per-tag TY → Distance tables
  // -------------------------------------------------------
  private static final double[][] T10_DISTANCE_DATA = {
    {3.77, 0.53}, // {tY, distance}
    {-3.61, 0.89},
    {-9.61, 1.34},
    {-14.42, 1.88},
    {-17.36, 2.63}
  };

  private static final double[][] T11_DISTANCE_DATA = {
    {3.77, 0.53}, // {tY, distance}
    {-3.61, 0.89},
    {-9.61, 1.34},
    {-14.42, 1.88},
    {-17.36, 2.63}
  };

  private static final double[][] T8_DISTANCE_DATA = {
    {3.77, 0.53}, // {tY, distance}
    {-3.61, 0.89},
    {-9.61, 1.34},
    {-14.42, 1.88},
    {-17.36, 2.63}
  };

  private static final double[][] T2_DISTANCE_DATA = {
    {3.77, 0.53}, // {tY, distance}
    {-3.61, 0.89},
    {-9.61, 1.34},
    {-14.42, 1.88},
    {-17.36, 2.63}
  };

  private static final double[][] T5_DISTANCE_DATA = {
    {3.77, 0.53}, // {tY, distance}
    {-3.61, 0.89},
    {-9.61, 1.34},
    {-14.42, 1.88},
    {-17.36, 2.63}
  };

  private static final double[][] T13_DISTANCE_DATA = {
    {3.77, 0.53}, // {tY, distance}
    {-3.61, 0.89},
    {-9.61, 1.34},
    {-14.42, 1.88},
    {-17.36, 2.63}
  };

  private static final double[][] T1_DISTANCE_DATA = { // Official
    {3.77, 0.53}, // {tY, distance}
    {-3.61, 0.89},
    {-9.61, 1.34},
    {-14.42, 1.88},
    {-17.36, 2.63}
  };

  private static final double[][] T12_DISTANCE_DATA = { // Official
    {3.77, 0.53}, // {tY, distance}
    {-3.61, 0.89},
    {-9.61, 1.34},
    {-14.42, 1.88},
    {-17.36, 2.63}
  };

  private static final double[][] T7_DISTANCE_DATA = {
    {-2.58, 0.33}, // {tY, distance}
    {-9.91, 0.65},
    {-16.05, 1.14},
    {-19.13, 1.62},
    {-20.49, 2.01},
  };

    private static final double[][] T6_DISTANCE_DATA = {
    {8.64, 0.05}, // {tY, distance}
    {-4.27, 0.35},
    {-11.01, 0.65},
    {-16.12, 1.16},
    {-19.97, 1.64},
    {-21.19, 2.13},
    {-22.22, 2.58}
  };

  // Distance → Shooter RPS
  private static final double[][] SPEED_DATA = {
    {1.51, 60},  // {distance, speed}
    {1.92, 62},
    {2.34, 64},
    {2.71, 67},
    {2.75, 68},
    {2.80, 70},
    {3.00, 70},
    {3.25, 70}
  };

  public LimelightSubsystem() {
    distanceToSpeedLookup.InsertValues(SPEED_DATA);
  }

  // -------------------------------------------------------
  //   Tag Classification
  // -------------------------------------------------------

  /** Returns the TY→Distance table for a given tag ID, or null if unsupported. */
  public double[][] getDataForTag(double IDNum) {
    switch ((int) Math.round(IDNum)) {
      case 10: return T10_DISTANCE_DATA;
      case 11: return T10_DISTANCE_DATA;
      case 8: return T10_DISTANCE_DATA;
      case 7: return T7_DISTANCE_DATA;
      case 12: return T7_DISTANCE_DATA;
      case 6: return T6_DISTANCE_DATA;
      case 1: return T6_DISTANCE_DATA;

      case 26: return T10_DISTANCE_DATA;
      case 27: return T10_DISTANCE_DATA;
      case 24: return T10_DISTANCE_DATA;
      case 28: return T7_DISTANCE_DATA;
      case 23: return T7_DISTANCE_DATA;
      case 17: return T6_DISTANCE_DATA;
      case 22: return T6_DISTANCE_DATA;
      default: return null;
    }
  }

  /** Tags that are valid targets for auto-aiming/rotation. */
  public boolean isAimTag() {
    int id = (int) Math.round(Variables.limelight.tID);
    return id == 2 || id == 5 || id == 10 || id == 13 || id == 12 || id == 1;
  }

  // -------------------------------------------------------
  //   Computed Values
  // -------------------------------------------------------

  /** Returns distance to the given tag ID using the current TY reading. */
  public double getTagDistance() {
    double[][] data = getDataForTag(Variables.limelight.tID);
    if (data == null) return 0;

    tagToDistanceLookup = new APTree();
    tagToDistanceLookup.InsertValues(data);
    return tagToDistanceLookup.GetValue(Variables.limelight.tY);
  }

  /** Returns the shooter RPS for the given tag ID based on distance. */
  public double getShooterRPS() {
    if (getDataForTag(Variables.limelight.tID) == null) return 30.0;
    return distanceToSpeedLookup.GetValue(Variables.limelight.distanceMeters);
  }

  public double getTurnAngle(double currentRobotHeadingDeg) {
    if (!Variables.limelight.hasValidTarget) {
        return currentRobotHeadingDeg;
    }

    double tx = Variables.limelight.tX;
    double sideOffsetDeg = 50;

    if (tx > 0) {
        // tag is to the right
        return Calculations.normalizeAngle360(currentRobotHeadingDeg - tx + sideOffsetDeg);
    } else {
        // tag is to the left
        return Calculations.normalizeAngle360(currentRobotHeadingDeg - tx - sideOffsetDeg);
    }
}

  // -------------------------------------------------------
  //   Raw Limelight Network Table Accessors
  // -------------------------------------------------------

  public double getDoubleEntry(String entry) {
    return limelight.getEntry(entry).getDouble(0);
  }

  public double[] getArrayEntry(String entry) {
    return limelight.getEntry(entry).getDoubleArray(new double[6]);
  }

  public Pose getPoseFromTag(double robotYawDeg) {

    if (!Variables.limelight.hasValidTarget) {
        return null; // or return current pose instead
    }

    double distance = Variables.limelight.distanceMeters + CAMERA_TO_CENTRE;  // already updating in periodic
    double tx = Variables.limelight.tX;

    // Bearing from field frame
    double bearingRad = Math.toRadians(robotYawDeg - tx);

    // Tag assumed at (0,0)
    double robotX = -distance * Math.cos(bearingRad);
    double robotY = -distance * Math.sin(bearingRad);

    return new Pose(robotX, robotY, robotYawDeg);
  }

  // -------------------------------------------------------
  //   Rumble Tags
  // -------------------------------------------------------

  public boolean isRumbleTagVisible() {
    if (!Variables.limelight.hasValidTarget) {
      return false;
    }

    int id = (int) Math.round(Variables.limelight.tID);
    return id == 10 || id == 8 || id == 24 || id == 27 || id == 26 || id == 11;
  }

  // -------------------------------------------------------
  //   Periodic
  // -------------------------------------------------------

  @Override
  public void periodic() {
    Variables.limelight.hasValidTarget = limelight.getEntry("tv").getDouble(0) == 1;
    SmartDashboard.putBoolean("HAS TARGET", Variables.limelight.hasValidTarget);

    if (Variables.limelight.hasValidTarget) {
        Variables.limelight.tID = getDoubleEntry("tid");
        Variables.limelight.tA  = getDoubleEntry("ta");
        Variables.limelight.tX  = getDoubleEntry("tx");
        Variables.limelight.tY  = getDoubleEntry("ty");

        Variables.limelight.distanceMeters = getTagDistance();
        Variables.limelight.turnAngle = getTurnAngle(Variables.drive.robotHeading);
        Variables.limelight.shooterRPS     = getShooterRPS();

        SmartDashboard.putNumber("tid", Variables.limelight.tID);
        SmartDashboard.putNumber("ta", Variables.limelight.tA);
        SmartDashboard.putNumber("ty", Variables.limelight.tY);
        SmartDashboard.putNumber("tx", Variables.limelight.tX);

        SmartDashboard.putNumber("distanceMeters", Variables.limelight.distanceMeters);
        SmartDashboard.putNumber("turnAngle", Variables.limelight.turnAngle);
        SmartDashboard.putNumber("shooterRPS", Variables.limelight.shooterRPS);
    }
  }
}