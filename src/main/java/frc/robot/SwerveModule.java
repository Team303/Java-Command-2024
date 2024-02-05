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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.subsystems.DriveSubsystem;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  public static final double kWheelDiameter = kWheelRadius * 2;
  private static final int kEncoderResolution = 42;

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

  public final CANcoder turningEncoder;

  public CANcoder getTurningEncoder() {
    return turningEncoder;
  }

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.2, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController = new PIDController(7, 0, 0);
  
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
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1.5, 0.2);

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
      CANcoderConfiguration config) {
    driveMotor = new TalonFX(driveMotorChannel);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
      
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    turningMotor.setIdleMode(IdleMode.kBrake);
      
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);

    driveMotor.getConfigurator().apply(clc);
    turningMotor.setSmartCurrentLimit(40);

    turningEncoder = new CANcoder(turningEncoderChannelA);
    turningEncoder.getConfigurator().apply(config);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    
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
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(), Rotation2d.fromRotations(turningEncoder.getAbsolutePosition().refresh().getValue() % 1.0));
  }


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
        m_drivePIDController.calculate(getDriveVelocity() * 2 * Math.PI * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR *  kWheelRadius, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(turningEncoder.getAbsolutePosition().refresh().getValue() % 1.0 * 2 * Math.PI, state.angle.getRadians());

    // final double turnFeedforward =
    //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    mainDriveOutput = driveOutput;

    mainTurnOutput = turnOutput;


    driveMotor.setVoltage(driveOutput + driveFeedforward);
    turningMotor.setVoltage(turnOutput);
  }
}
