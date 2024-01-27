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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.sensors.CANCoder;

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
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final CANCoder turningEncoder;

  public CANCoder getTurningEncoder() {
    return turningEncoder;
  }

  public RelativeEncoder getDriveEncoder() {
    return driveEncoder;
  }

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(2, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController = new PIDController(8, 0, 0);
  
  /*  =
      new ProfiledPIDController(
          2,
          0,
          0.2,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity*10 , kModuleMaxAngularAcceleration * 50));
  */

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0,3.12, 0.36);
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
      int turningEncoderChannelA) {
    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = new CANCoder(turningEncoderChannelA);

    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);

    driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR);
    driveEncoder.setVelocityConversionFactor(2 * Math.PI * kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR);

    driveMotor.enableVoltageCompensation(12);
    driveMotor.setSmartCurrentLimit(40);

    turningMotor.enableVoltageCompensation(12);
    turningMotor.setSmartCurrentLimit(40);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    turningEncoder.setPositionToAbsolute();
  }

  public double getDriveCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public void invertSteerMotor(boolean inversion) {
    turningMotor.setInverted(inversion);
  }

  public void invertDriveMotor(boolean inversion) {
    driveMotor.setInverted(inversion);
  }

  /**\
   * Returns the drive motor
   * @return The CANSparkMax of the drive motor
   */
  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), Rotation2d.fromDegrees(turningEncoder.getPosition() % 360));
  }

  /**
   * Returns the current Velocity of the module in m/s
   * @return the current Velocity of the module in m/s
   */
  public double getDriveVelocity() {
    return driveEncoder.getVelocity(); // / 60 * 2 * Math.PI * kWheelRadius;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), Rotation2d.fromDegrees(turningEncoder.getPosition() % 360));
  }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d encoderRotation = Rotation2d.fromDegrees(turningEncoder.getPosition() % 360);


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(turningEncoder.getPosition() % 360));

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller. Divide by 60 to get Meters per Second
    final double driveOutput =
        m_drivePIDController.calculate(driveEncoder.getVelocity() / 60, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(Math.toRadians(turningEncoder.getPosition() % 360), state.angle.getRadians());

    // final double turnFeedforward =
    //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    mainDriveOutput = driveOutput;

    mainTurnOutput = turnOutput;

    driveMotor.setVoltage(driveOutput + driveFeedforward);
    turningMotor.setVoltage(turnOutput);
  }
}
