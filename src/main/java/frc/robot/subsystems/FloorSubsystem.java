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

public class FloorSubsystem extends SubsystemBase {
  private final TalonFX topFloorMotor;
  private final TalonFX bottomFloorMotor; 
  private final VelocityVoltage velocityRequest;

  /** Creates a new floorSubsytem. */
  public FloorSubsystem() {
    topFloorMotor = new TalonFX(31);
    bottomFloorMotor = new TalonFX(32);
    
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    topFloorMotor.getConfigurator().apply(Configs.floorMotor.floorConfig);

    bottomFloorMotor.setControl(   
      new Follower(topFloorMotor.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  public double getfloorSpeed () {
    return topFloorMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed () {
    return Math.abs(getfloorSpeed() - Variables.floor.floorRPS) < 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    topFloorMotor.setControl(velocityRequest.withVelocity(Variables.floor.floorRPS));
  }
}