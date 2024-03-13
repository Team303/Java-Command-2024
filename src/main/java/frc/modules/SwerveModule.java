// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap;
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

  // TODO: figure this out
  private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final TalonFX driveMotor;

  private final CANSparkMax turningMotor;

  private final VelocityVoltage voltageVelocityDriveControl = new VelocityVoltage(0, 10, true, 0, 0, true, false,
      false);

  private final VelocityTorqueCurrentFOC torqueVelocityDriveControl = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false,
      false, false);

  public final CANcoder turningEncoder;

  public CANcoder getTurningEncoder() {
    return turningEncoder;
  }

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(2, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  public final SparkPIDController m_turnPidController;

  /*
   * =
   * new ProfiledPIDController(
   * 2,
   * 0,
   * 0.2,
   * new TrapezoidProfile.Constraints(
   * kModuleMaxAngularVelocity*10 , kModuleMaxAngularAcceleration * 50));
   */

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 2.35, 0.19);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0.07);

  public final RelativeEncoder turningNeoEncoder;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel      PWM output for the drive motor.
   * @param turningMotorChannel    PWM output for the turning motor.
   * @param driveEncoderChannelA   DIO input for the drive encoder channel A
   * @param driveEncoderChannelB   DIO input for the drive encoder channel B
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

    driveMotor.setPosition(0);
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);

    driveMotor.getConfigurator().apply(clc);
    turningMotor.setSmartCurrentLimit(40);

    turningEncoder = new CANcoder(turningEncoderChannelA);
    turningEncoder.getConfigurator().apply(config);

    turningNeoEncoder = turningMotor.getEncoder();

    turningNeoEncoder.setPositionConversionFactor(
        Swerve.STEER_REDUCTION * Math.PI * 2);
    turningNeoEncoder.setVelocityConversionFactor(
        Swerve.STEER_REDUCTION * Math.PI * 2);
    turningNeoEncoder.setPosition(getPosition().angle.getRadians());

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Slot0.kP = 0.1; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.; // A change of 1 rotation per second squared results in 0.01 volts output
    // configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333
    // rps per V, 1/8.33 = 0.12
    // volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    driveMotor.getConfigurator().apply(configs);

    m_turnPidController = turningMotor.getPIDController();

    m_turnPidController.setP(3);
    m_turnPidController.setI(0);
    m_turnPidController.setD(0);
    m_turnPidController.setFF(0.01);
    m_turnPidController.setFeedbackDevice((MotorFeedbackSensor) turningNeoEncoder);
    m_turnPidController.setPositionPIDWrappingMaxInput(Math.PI);
    m_turnPidController.setPositionPIDWrappingMinInput(-Math.PI);
    m_turnPidController.setPositionPIDWrappingEnabled(true);
    m_turnPidController.setSmartMotionAllowedClosedLoopError(Math.toRadians(1.5), 0);

    // turningEncoder.setPositionToAbsolute();
  }

  public void invertSteerMotor(boolean inversion) {
    turningMotor.setInverted(inversion);
  }

  public void invertDriveMotor(boolean inversion) {
    driveMotor.setInverted(inversion);
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().refresh().getValue() * 2 * Math.PI * kWheelRadius
        * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR;
  }

  /**
   * \
   * Returns the drive motor
   * 
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
   * 
   * @return the current Velocity of the module in m/s
   */
  public double getDriveVelocity() {
    return driveMotor.getVelocity().refresh().getValue() * 2 * Math.PI * kWheelRadius
        * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR; // / 60 * 2 * Math.PI * kWheelRadius;
  }

  /**
   * Returns the current Velocity of the steer motor in m/s
   * 
   * @return the current Velocity of the steer motor in m/s
   */
  public double getTurnVelocity() {
    return turningEncoder.getVelocity().refresh().getValue() * 2 * Math.PI;// * kWheelRadius *
                                                                           // RobotMap.Swerve.SWERVE_CONVERSION_FACTOR;
                                                                           // // / 60 * 2 * Math.PI * kWheelRadius;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(),
        Rotation2d.fromRotations(turningEncoder.getAbsolutePosition().refresh().getValueAsDouble()));
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

  final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(2);
  final double ENCODER_RESET_ITERATIONS = 200;
  double resetIteration = 0;

  public void periodicReset() {
    // System.out.println("resetIteration: " + resetIteration);
    // System.out.println("turning velocity: " + turningNeoEncoder.getVelocity());
    // System.out.println("max vel: " + ENCODER_RESET_MAX_ANGULAR_VELOCITY);
    if (Math.abs(turningNeoEncoder.getVelocity()) < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {

      if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        resetIteration = 0;
        System.out.println("resetting positions to CANCoders");
        double absoluteAngle = normalizeAngle(getPosition().angle.getRadians());
        turningNeoEncoder.setPosition(absoluteAngle);
      }
    } else {
      resetIteration = 0;
    }
  }

  public static double normalizeAngle2(double angleRadians) {

    angleRadians %= Math.PI * 2;

    if (angleRadians >= Math.PI) {
      angleRadians -= Math.PI * 2;
    }

    if (angleRadians <= -Math.PI) {
      angleRadians += Math.PI * 2;
    }

    return angleRadians;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        Rotation2d.fromRadians(normalizeAngle2(turningNeoEncoder.getPosition())));

    // Logger.recordOutput("desired drive velocity", state.speedMetersPerSecond /
    // (2* Math.PI*kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR));

    driveMotor.setControl(voltageVelocityDriveControl
        .withVelocity(
            state.speedMetersPerSecond / (2 * Math.PI * kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR))
        .withAcceleration(10));

    turningMotor.getPIDController().setReference(normalizeAngle2(state.angle.getRadians()),
        CANSparkMax.ControlType.kPosition);

  }
}
