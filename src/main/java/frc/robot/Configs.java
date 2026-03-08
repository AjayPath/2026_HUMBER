package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.Amps;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            
            // Current Limits - ADDED STATOR LIMIT
            drivingConfig.CurrentLimits.SupplyCurrentLimit = 50;
            drivingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            drivingConfig.CurrentLimits.StatorCurrentLimit = 70;  // NEW - swerve drive under load
            drivingConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            
            drivingConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            drivingConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction;

            drivingConfig.Slot0.kP = 0.1;
            drivingConfig.Slot0.kI = 0.0;
            drivingConfig.Slot0.kD = 0.0;
            drivingConfig.Slot0.kV = drivingVelocityFeedForward;

            drivingConfig.Voltage.PeakForwardVoltage = 12.0;
            drivingConfig.Voltage.PeakReverseVoltage = -12.0;

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // contintake to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class shooterMotor {

        public static final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        static {
            // ========== LEFT SHOOTER CONFIG ==========
            shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            // Current Limits
            shooterConfig.CurrentLimits.SupplyCurrentLimit = 120;
            shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            shooterConfig.CurrentLimits.StatorCurrentLimit = 80;  // Prevents overheating
            shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

            // Invert Motor
            shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            // Gear Ratio
            shooterConfig.Feedback.SensorToMechanismRatio = 1.0;

            // PID VALUES
            shooterConfig.Slot0.kP = 0.2;
            shooterConfig.Slot0.kI = 0.0;
            shooterConfig.Slot0.kD = 0.0;
            shooterConfig.Slot0.kV = 12.0 / Constants.KrakenX60.kFreeSpeedRPS;

            // Voltage Control
            shooterConfig.Voltage.PeakForwardVoltage = 12.0;
            shooterConfig.Voltage.PeakReverseVoltage = -12.0;
        }
    }

    public static final class feederMotor {        
        // Create configuration class for our feeder motors
        public static final TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration feederVoltageConfig = new TalonFXConfiguration();

        static {
            // Coast or Brake
            feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            // Current Limits - ADDED STATOR LIMIT
            feederConfig.CurrentLimits.SupplyCurrentLimit = 60;
            feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            feederConfig.CurrentLimits.StatorCurrentLimit = 60;  // NEW - medium load mechanism
            feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

            // Invert Motor
            feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            // Gear Ratio
            feederConfig.Feedback.SensorToMechanismRatio = 1.0;

            // PID VALUES
            feederConfig.Slot0.kP = 0.1;
            feederConfig.Slot0.kI = 0.0;
            feederConfig.Slot0.kD = 0.0;
            feederConfig.Slot0.kV = 12.0 / Constants.KrakenX60.kFreeSpeedRPS;

            // Voltage Control
            feederConfig.Voltage.PeakForwardVoltage = 12.0;
            feederConfig.Voltage.PeakReverseVoltage = -12.0;

            feederVoltageConfig.withMotorOutput(
                 new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
                        .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
            );
        }
    }

    public static final class floorMotor {

        public static final TalonFXConfiguration floorConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration floorVoltageConfig = new TalonFXConfiguration();
        
        static {
            floorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            // Current Limits - ADDED STATOR LIMIT
            floorConfig.CurrentLimits.SupplyCurrentLimit = 40;
            floorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            floorConfig.CurrentLimits.StatorCurrentLimit = 60;  // NEW - medium load mechanism
            floorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

            floorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
            floorConfig.Feedback.SensorToMechanismRatio = 1.0;

            floorConfig.Slot0.kP = 0.1;
            floorConfig.Slot0.kI = 0.0;
            floorConfig.Slot0.kD = 0.0;
            floorConfig.Slot0.kV = 12.0 / Constants.KrakenX60.kFreeSpeedRPS;

            // Voltage Control  
            floorConfig.Voltage.PeakForwardVoltage = 12.0;
            floorConfig.Voltage.PeakReverseVoltage = -12.0;


            floorVoltageConfig.withMotorOutput(
                 new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
                        .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30))
                    .withSupplyCurrentLimitEnable(true)
            );
        }
    }

    public static final class intakeMotor {        
        // Create configuration class for our intake motors
        public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        static {
            // ── Intake ───────────────────────────────────────────────────────────────
            // Coast or Brake
            intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            // Current Limits - ADDED STATOR LIMIT
            intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
            intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            intakeConfig.CurrentLimits.StatorCurrentLimit = 60;  // NEW - medium load mechanism
            intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

            // Invert Motor
            intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            // Gear Ratio
            intakeConfig.Feedback.SensorToMechanismRatio = 1.0;

            // PID VALUES
            intakeConfig.Slot0.kP = 0.1;
            intakeConfig.Slot0.kI = 0.0;
            intakeConfig.Slot0.kD = 0.0;
            intakeConfig.Slot0.kV = 12.0 / Constants.KrakenX60.kFreeSpeedRPS;

            // Voltage Control
            intakeConfig.Voltage.PeakForwardVoltage = 12.0;
            intakeConfig.Voltage.PeakReverseVoltage = -12.0;
        }
    }

    public static final class pivotMotor {        
        public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        public static final double PIVOT_GEAR_RATIO = 32.0;

        static {
        // ── Pivot ───────────────────────────────────────────────────────────────
            pivotConfig.CurrentLimits.SupplyCurrentLimit = 50;
            pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
            pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

            pivotConfig.Slot0.kV = 12.0 / Constants.KrakenX60.kFreeSpeedRPS;
            pivotConfig.Slot0.kP = 30.0;
            pivotConfig.Slot0.kI = 0.0;
            pivotConfig.Slot0.kD = 0.0;

            pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
            pivotConfig.MotionMagic.MotionMagicAcceleration = 160;
            pivotConfig.MotionMagic.MotionMagicJerk = 1600;

            pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 120.0 / 360.0 * PIVOT_GEAR_RATIO;
            pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

            pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
    }

}