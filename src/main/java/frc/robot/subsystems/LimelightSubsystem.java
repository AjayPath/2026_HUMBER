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
    {6.00, 0.298}, // {tY, distance}
    {-2.14, 0.525},
    {-6.89, 0.720},
    {-10.59, 0.930},
    {-13.37, 1.105},
    {-16.59, 1.380},
    {-18.95, 1.635},
    {-20.69, 1.905},
    {-21.96, 2.143},
    {-23.65, 2.530},
    {-24.48, 2.728},
  };

  private static final double[][] T8_DISTANCE_DATA = {
    {4.00, 0.32}, // {tY, distance}
    {-2.65, 0.52},
    {-10.00, 0.86},
    {-13.51, 1.09},
    {-15.84, 1.28},
    {-18.65, 1.57},
    {-20.96, 1.85},
    {-22.69, 2.20},
    {-23.36, 2.45},
    {-24.03, 2.69},
    {-25.00, 2.74},
    {-26.00, 2.76},
    {-28.00, 2.80},
    {-29.00, 2.90},
    {-30.00, 3.00}
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

  private static final double[][] T7_DISTANCE_DATA = {
    {5.88, 0.00}, // {tY, distance}
    {-3.89, 0.22},
    {-11.13, 0.41},
    {16.67, 0.68},
    {-20.17, 0.95},
    {-22.65, 1.21},
    {-24.42, 1.47},
    {-26.24, 1.98},
  };

    private static final double[][] T6_DISTANCE_DATA = {
    {-8.26, 0.30}, // {tY, distance}
    {-14.27, 0.55},
    {-18.83, 0.90},
    {-22.51, 1.19},
    {-25.28, 1.49},
    {-26.15, 1.72},
  };

  // Distance → Shooter RPS
  private static final double[][] SPEED_DATA = {
    {1.51, 60},  // {distance, speed}
    {1.92, 62},
    {2.34, 64},
    {2.71, 67},
    {2.75, 69},
    {2.80, 72},
    {3.00, 75},
    {3.25, 75}
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
      case 8: return T8_DISTANCE_DATA;
      case 11: return T8_DISTANCE_DATA;
      case 7: return T7_DISTANCE_DATA;
      case 6: return T6_DISTANCE_DATA;
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