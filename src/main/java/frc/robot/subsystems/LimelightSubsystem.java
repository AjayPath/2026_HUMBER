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
    {-19.06, 0.71}, // {tY, distance}
    {-21, 1},
    {-22, 1.5},
  };

  private static final double[][] T2_DISTANCE_DATA = {
    {4.93 ,1.30}, // {tY, distance}
    {4.15, 1.40},
    {2.37, 1.52},
    {0.20, 1.73},
    {-1.86, 1.93},
    {-3.60, 2.20},
    {-4.97, 2.50},
    {-6.00, 2.74},
    {-6.51, 2.90},
    {-7.22, 3.07},
  };

  private static final double[][] T5_DISTANCE_DATA = {
    {21.10, 10.93}, // {tY, distance}
    {10, 0},
  };

  private static final double[][] T13_DISTANCE_DATA = {
    {-15.91, 0.70}, // {tY, distance}
    {-16.53, 0.94},
    {-16.71, 1.10},
    {-16.88, 1.23},
    {-17.00, 1.42},
    {-17.18, 1.64},
    {-17.29, 2.00},
    {-17.31, 2.30},
    {-17.36, 2.53},
    {-17.38, 2.90},
    {-17.43, 3.00},
  };

  private static final double[][] T1_DISTANCE_DATA = { // Official
    {14.98, 0.52}, // {tY, distance}
    {4.87, 0.81},
    {2.10, 0.94},
    {-1.02, 1.20},
    {-4.00, 1.40},
    {-6.12, 1.69},
    {-7.77, 1.96},
    {-9.34, 2.35},
    {-10.67, 2.74},
    {-11.62 ,3.00},
    {-12.41, 3.35},
  };

  private static final double[][] T12_DISTANCE_DATA = { // Official
    {16.78, 0.52}, // {tY, distance}
    {7.14, 0.80},
    {2.16, 0.97},
    {-1.41, 1.20},
    {-5.12, 1.54},
    {-7.24, 1.86},
    {-8.72, 2.17},
  };

  // Distance → Shooter RPS
  private static final double[][] SPEED_DATA = {
    {1.67, 56}, // {distance, speed}
    {1.79, 58},
    {1.92, 59},
    {2.10, 60},
    {2.20, 62},
    {2.40, 62},
    {2.58, 63},
    {2.90, 67},
    {3.07, 68},
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
      case 2:  return T2_DISTANCE_DATA;
      case 5:  return T5_DISTANCE_DATA;
      case 13: return T13_DISTANCE_DATA;
      case 1:  return T1_DISTANCE_DATA;
      case 12: return T12_DISTANCE_DATA;
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