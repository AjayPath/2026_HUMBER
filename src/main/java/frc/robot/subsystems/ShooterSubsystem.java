// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Variables;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX rightShooterMotor;
  private final TalonFX leftShooterMotor; 
  private final VelocityVoltage velocityRequest;

  /** Creates a new ShooterSubsytem. */
  public ShooterSubsystem() {
    rightShooterMotor = new TalonFX(40);
    leftShooterMotor = new TalonFX(41);
    
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    rightShooterMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);

    leftShooterMotor.setControl(   
      new Follower(rightShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed)
    );
  }

  public double getRightShooterSpeed () {
    return rightShooterMotor.getVelocity().getValueAsDouble();
  }

    public double getLeftShooterSpeed () {
    return rightShooterMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed () {
    return (Math.abs(getRightShooterSpeed() - Variables.shooter.shooterRPS) < 1) && (Math.abs(getLeftShooterSpeed() - Variables.shooter.shooterRPS) < 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightShooterMotor.setControl(velocityRequest.withVelocity(Variables.shooter.shooterRPS));
  }
}