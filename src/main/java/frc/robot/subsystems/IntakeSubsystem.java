// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Variables;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final VelocityVoltage velocityRequest;

  /** Creates a new IntakeSubsytem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(30);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    intakeMotor.getConfigurator().apply(Configs.intakeMotor.intakeConfig);
  }

  public double getintakeSpeed () {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed () {
    return Math.abs(getintakeSpeed() - Variables.intake.intakeRPS) < 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeMotor.setControl(velocityRequest.withVelocity(Variables.intake.intakeRPS));
  }
}