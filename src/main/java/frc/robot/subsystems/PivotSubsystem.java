package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Variables;

public class PivotSubsystem extends SubsystemBase {
  private final TalonFX pivotMotor;
  private final MotionMagicVoltage pivotRequest;

  public PivotSubsystem() {
    pivotMotor = new TalonFX(50);
    pivotRequest = new MotionMagicVoltage(0).withSlot(0);

    pivotMotor.getConfigurator().apply(Configs.pivotMotor.pivotConfig);

    pivotMotor.setPosition(0);
  }

  public void setPivotAngle(double degrees) {
    double rotations = degrees / 360.0 * Configs.pivotMotor.PIVOT_GEAR_RATIO;
    pivotMotor.setControl(pivotRequest.withPosition(rotations));
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  public double getPivotAngle() {
    return (pivotMotor.getPosition().getValueAsDouble() / Configs.pivotMotor.PIVOT_GEAR_RATIO) * 360.0;
  }

  public boolean pivotAtTarget(double targetDegrees, double toleranceDegrees) {
    return Math.abs(getPivotAngle() - targetDegrees) < toleranceDegrees;
  }

  @Override
  public void periodic() {
    setPivotAngle(Variables.pivot.pivotPosition);
  }
}