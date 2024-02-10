// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.subsystems.DriveSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;
import frc.robot.RobotMap.Swerve;
import frc.subsystems.DriveSubsystem;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  public static final double kWheelDiameter = kWheelRadius * 2;
  private static final int kEncoderResolution = 42;

  private double lastTurnVelocity = 0;


  private double mainDriveOutput;
  
  private double mainTurnOutput;
  
  public double getMainDriveOutput() {
    return mainDriveOutput;
  }
  public double getMainTurnOutput() {
    return mainTurnOutput;
  }

  //TODO: figure this out
  private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final TalonFX driveMotor;

  private final CANSparkMax turningMotor;

  private final VelocityVoltage voltageVelocityDriveControl = new VelocityVoltage(0, 10, true, 0, 0, true, false, false);

  private final VelocityTorqueCurrentFOC torqueVelocityDriveControl = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

  public final CANcoder turningEncoder;

  public CANcoder getTurningEncoder() {
    return turningEncoder;
  }

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.2, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  public final PIDController m_turningPIDController;
  
  /*  =
      new ProfiledPIDController(
          2,
          0,
          0.2,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity*10 , kModuleMaxAngularAcceleration * 50));
  */

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0,2.35,0.19);  
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0.07);

  private final RelativeEncoder neoEncoder;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannelA,
      CANcoderConfiguration config,
      PIDController turnController
      ) {

    m_turningPIDController = turnController;
    driveMotor = new TalonFX(driveMotorChannel);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
      
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    turningMotor.setIdleMode(IdleMode.kBrake);
    
    driveMotor.setPosition(0);
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);

    driveMotor.getConfigurator().apply(clc);
    turningMotor.setSmartCurrentLimit(40); 

    neoEncoder = turningMotor.getEncoder();
    turningEncoder = new CANcoder(turningEncoderChannelA);
    turningEncoder.getConfigurator().apply(config);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    
    configs.Slot0.kP = 0.1; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    driveMotor.getConfigurator().apply(configs);

    
    // turningEncoder.setPositionToAbsolute();
  }

  public void invertSteerMotor(boolean inversion) {
    turningMotor.setInverted(inversion);
  }

  public void invertDriveMotor(boolean inversion) {
    driveMotor.setInverted(inversion);
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().refresh().getValue() * 2 * Math.PI * kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR;
  }

  /**\
   * Returns the drive motor
   * @return The TalonFX of the drive motor
   */
  public TalonFX getDriveMotor() {
    return driveMotor;
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(), Rotation2d.fromRotations(turningEncoder.getAbsolutePosition().refresh().getValue() % 1.0));
  }

  /**
   * Returns the current Velocity of the module in m/s
   * @return the current Velocity of the module in m/s
   */
  public double getDriveVelocity() {
    return driveMotor.getVelocity().refresh().getValue() * 2 * Math.PI * kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR; // / 60 * 2 * Math.PI * kWheelRadius;
  }

    /**
   * Returns the current Velocity of the steer motor in m/s
   * @return the current Velocity of the steer motor in m/s
   */
  public double getTurnVelocity() {
    return turningEncoder.getVelocity().refresh().getValue() * 2 * Math.PI;// * kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR; // / 60 * 2 * Math.PI * kWheelRadius;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(), Rotation2d.fromRotations(turningEncoder.getAbsolutePosition().refresh().getValueAsDouble()));
  }

  /**
   * Normalizes angle value to be inbetween values 0 to 2pi.
   *
   * @param angle angle to be normalized
   * @return angle value between 0 to 2pi
   */
  private double normalizeAngle(double angle) {
    angle %= (2.0 * Math.PI);
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }

  /**
   * Returns difference (targetAngle - getSteerAngle())
    normalized in range -pi .. pi
   *
   * @param targetAngle the angle to be moved to
   * @return The steer angle after accounting for error.
   */
  public double normalizeAngleError(double targetAngle) {
    // Angle is inbetween 0 to 2pi

    double difference = targetAngle - getPosition().angle.getRadians();
    // Change the target angle so the difference is in the range [-pi, pi) instead
    // of [0, 2pi)
    if (difference >= Math.PI) {
      targetAngle -= 2.0 * Math.PI;
    } else if (difference < -Math.PI) {
      targetAngle += 2.0 * Math.PI;
    } 
    return targetAngle - getPosition().angle.getRadians();
  }

  double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5) / (2 * Math.PI);
  double ENCODER_RESET_ITERATIONS = 100;
  double resetIteration = 0;

    /**
   * Converts the steer angle to the next angle the swerve module should turn to.
   *
   * @param steerAngle the current steer angle.
   */
  private double convertSteerAngle(double steerAngle) {
    steerAngle = normalizeAngle(steerAngle);
    double difference = normalizeAngleError(steerAngle);        

    // If the difference is greater than 90 deg or less than -90 deg the drive can
    // be inverted so the total
    // movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into
      // the range [0, 2pi)
      steerAngle += Math.PI;
    }

    // Put the target angle back into the range [0, 2pi)
    steerAngle = normalizeAngle(steerAngle);

    // Angle to be changed is now in radians
    double referenceAngleRadians = steerAngle;

    double currentAngleRadians = turningMotor.getEncoder().getPosition();

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
    // fully set up, and we don't
    // end up getting a good reading. If we reset periodically this won't matter
    // anymore.
    if (turningMotor.getEncoder().getVelocity() 
            < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
      if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        resetIteration = 0;
        double absoluteAngle = getPosition().angle.getRadians();    
        turningMotor.getEncoder().setPosition(getPosition().angle.getRadians());
        currentAngleRadians = absoluteAngle;
      }
    } else {
      resetIteration = 0;
    }

    double currentAngleRadiansMod = normalizeAngle(currentAngleRadians);

    // The reference angle has the range [0, 2pi)
    // but the Falcon's encoder can go above that
    double adjustedReferenceAngleRadians = referenceAngleRadians 
        + currentAngleRadians - currentAngleRadiansMod;
    if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }

    // The position that the motor should turn to
    // when taking into account the ticks of the motor
    return adjustedReferenceAngleRadians;
  }

  private double normalizeAngle2(double angle) {
    angle %= 2 * Math.PI;
    if (Math.abs(angle) < Math.PI)
      return angle;
    else if (angle > 0)
      return angle - Math.PI * 2;
    else 
      return angle + Math.PI * 2;
  }

  /**
   * Converts the drive votlage to be inverted or not.
   *
   * @

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(turningEncoder.getAbsolutePosition().refresh().getValue() % 1.0));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(convertSteerAngle(turningEncoder.getAbsolutePosition().refresh().getValue() * 2 * Math.PI), state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(getTurnVelocity());

    mainDriveOutput = driveOutput;
    mainTurnOutput = turnOutput;

    
    //driveMotor.setControl(voltageVelocityDriveControl.withVelocity((state.speedMetersPerSecond / (2* Math.PI*0.0508 * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR))));

    // driveMotor.setControl(torqueVelocityDriveControl.withVelocity(1000));


    Logger.recordOutput("desired drive velocity", state.speedMetersPerSecond / (2* Math.PI*kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR));

    //  if (Math.abs(Robot.controller.getLeftX()) < 0.2 && Math.abs(Robot.controller.getLeftY()) < 0.2 && Math.abs(Robot.controller.getRightX()) < 0.05) {
    //  } 
    // else {
    driveMotor.setControl(voltageVelocityDriveControl.withVelocity(state.speedMetersPerSecond / (2* Math.PI*kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR)).withAcceleration(10));
    //  }
    //driveMotor.setControl(voltageVelocityDriveControl.withVelocity(0));
    // driveMotor.setVoltage(driveOutput + driveFeedforward);
    turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
