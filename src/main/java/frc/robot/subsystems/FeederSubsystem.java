// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Variables;

public class FeederSubsystem extends SubsystemBase {
  private final TalonFX feederMotor;
  private final VelocityVoltage velocityRequest;

  /** Creates a new feederSubsytem. */
  public FeederSubsystem() {
    feederMotor = new TalonFX(33);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    feederMotor.getConfigurator().apply(Configs.feederMotor.feederConfig);
  }

  public double getfeederSpeed () {
    return feederMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed () {
    return Math.abs(getfeederSpeed() - Variables.feeder.feederRPS) < 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feederMotor.setControl(velocityRequest.withVelocity(Variables.feeder.feederRPS));
  }
}