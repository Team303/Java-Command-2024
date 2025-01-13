// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Swerve;
import frc.subsystems.DriveSubsystem;

public class SwerveModule {
  public String name;
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

  private final TalonFX turningMotor;

  private final VelocityVoltage voltageVelocityDriveControl = new VelocityVoltage(0).withAcceleration(10).withEnableFOC(true).withFeedForward(0).withSlot(0).withOverrideBrakeDurNeutral(true);
  private final PositionVoltage positionAngleControl = new PositionVoltage(0).withVelocity(10).withEnableFOC(true).withFeedForward(0).withSlot(0).withOverrideBrakeDurNeutral(true);


  public final CANcoder turningCancoder;

  public CANcoder getTurningCancoder() {
    return turningCancoder;
  }

  // public final SparkClosedLoopController m_turnPidController;
  


  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel      PWM output for the drive motor.
   * @param turningMotorChannel    PWM output for the turning motor.
   * @param driveCanBus            String identifier for drive motor for CAN Bus (either "rio" for RoboRio or whatever the name of the CANivore is)
   * @param turnCanBus             String identifier for turning motor for CAN Bus (either "rio" for RoboRio or whatever the name of the CANivore is)
   * @param driveEncoderChannelA   DIO input for the drive encoder channel A
   * @param driveEncoderChannelB   DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(String position,
      int driveMotorChannel,
      int turningMotorChannel,
      String driveCanBus,
      String turnCanBus,
      int turningEncoderChannelA,
      CANcoderConfiguration config) {

    name = position;

    driveMotor = new TalonFX(driveMotorChannel,driveCanBus);
    turningMotor = new TalonFX(turningMotorChannel,turnCanBus);
    //Pray we do not need this line of code
    // m_turnPidController.setSmartMotionAllowedClosedLoopError(Math.toRadians(1.5), 0);
      
    driveMotor.setNeutralMode(NeutralModeValue.Coast);
    turningMotor.setNeutralMode(NeutralModeValue.Coast);

    driveMotor.setPosition(0);
    //  CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);

    //driveMotor.getConfigurator().apply(clc);
    // turningMotor.setSmartCurrentLimit(40);

    turningCancoder = new CANcoder(turningEncoderChannelA,turnCanBus);
    turningCancoder.getConfigurator().apply(config);

    
   // turningMotor.setPosition(turningCancoder.getAbsolutePosition().refresh().getValueAsDouble() % 1.0);

    TalonFXConfiguration driveConfigs = new TalonFXConfiguration();

    // TODO: tune PID valued for comp so no annoying af oscillations
    driveConfigs.Slot0.kP = 0.1; // An error of 1 rotation per second results in 2V output
    driveConfigs.Slot0.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
    driveConfigs.Slot0.kD = 0.; // A change of 1 rotation per second squared results in 0.01 volts output
    // configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333
    // rps per V, 1/8.33 = 0.12
    // volts / Rotation per second
    // Peak output of 8 volts
    driveConfigs.Voltage.PeakForwardVoltage = 12;
    driveConfigs.Voltage.PeakReverseVoltage = -12;

    driveMotor.getConfigurator().apply(driveConfigs);


    TalonFXConfiguration turnConfigs = new TalonFXConfiguration();
    turnConfigs.Feedback.FeedbackRemoteSensorID = turningCancoder.getDeviceID();
    turnConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    turnConfigs.Slot0.kP=30;
    turnConfigs.Slot0.kI=0;
    turnConfigs.Slot0.kD=0;
    // turnConfigs.Slot0.kV=1;

    //25
    //1.5
    //0
    //1.5
    
    //This was the ff value with revlib
    // turnConfigs.Slot0.kA=1.5;
    // turnConfigs.Slot0.kV=1;
    // turnConfigs.Slot0.kS=2.5;
    turnConfigs.ClosedLoopGeneral.ContinuousWrap=true;
    turnConfigs.Feedback.SensorToMechanismRatio=1;

    turningMotor.getConfigurator().apply(turnConfigs);

    var driveTalonFXConfigurator = driveMotor.getConfigurator();
    var turnTalonFXConfigurator = turningMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 130;
    limitConfigs.StatorCurrentLimitEnable = true;

    limitConfigs.SupplyCurrentLimit = 50;
    limitConfigs.SupplyCurrentLimitEnable = true;

    driveTalonFXConfigurator.apply(limitConfigs);
    turnTalonFXConfigurator.apply(limitConfigs);

    // turningEncoder.setPositionToAbsolute();
  }

  public void invertSteerMotor(boolean inversion) {
    turningMotor.setInverted(inversion);

  }

  public void invertDriveMotor(boolean inversion) {
    driveMotor.setInverted(inversion);
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().refresh().getValueAsDouble() * 2 * Math.PI * kWheelRadius
        * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR;
  }
  public double getTurnPosition() {
    return turningCancoder.getAbsolutePosition().refresh().getValueAsDouble();
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

  public TalonFX getTurnMotor() {
    return turningMotor;
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), Rotation2d.fromRotations(turningCancoder.getAbsolutePosition().refresh().getValueAsDouble() % 1.0));
        
  }

  /**
   * Returns the current Velocity of the module in m/s
   * 
   * @return the current Velocity of the module in m/s
   */
  public double getDriveVelocity() {
    return driveMotor.getVelocity().refresh().getValueAsDouble() * 2 * Math.PI * kWheelRadius
        * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR; // / 60 * 2 * Math.PI * kWheelRadius;
  }

  /**
   * Returns the current Velocity of the steer motor in m/s
   * 
   * @return the current Velocity of the steer motor in m/s
   */
  public double getTurnVelocity() {
    return turningCancoder.getVelocity().refresh().getValueAsDouble() * 2 * Math.PI;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(),
        Rotation2d.fromRotations(turningCancoder.getAbsolutePosition().refresh().getValueAsDouble()));
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

  // // ~5 percent of the time, CANCoders are not configured upon initialization blah
  // // blah blah

  public void periodicReset() {
    // System.out.println("resetIteration: " + resetIteration);
    // System.out.println("turning velocity: " + turningNeoEncoder.getVelocity());
    // System.out.println("max vel: " + ENCODER_RESET_MAX_ANGULAR_VELOCITY);
    if (Math.abs(turningMotor.getVelocity().getValueAsDouble()) < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {

      if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        resetIteration = 0;
        System.out.println("resetting positions to CANCoders");

        // removed normalize angle so it is -Math.PI to Math.PI
        double absoluteAngle = turningCancoder.getAbsolutePosition().refresh().getValueAsDouble() % 1.0;
        turningMotor.setPosition(absoluteAngle);
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
    desiredState.optimize(Rotation2d.fromRadians(normalizeAngle2(Units.rotationsToRadians(turningMotor.getPosition().refresh().getValueAsDouble()))));

    //turningMotor.setPosition(turningCancoder.getAbsolutePosition().refresh().getValueAsDouble() % 1.0);
    // Logger.recordOutput("desired drive velocity", state.speedMetersPerSecond /
    // (2* Math.PI*kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR));
    if(Math.abs(driveMotor.getVelocity().getValueAsDouble()- (desiredState.speedMetersPerSecond *2)/(2 * Math.PI * kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR))>0.3){
      driveMotor.setControl(voltageVelocityDriveControl
        .withVelocity(
            (desiredState.speedMetersPerSecond *2)/ (2 * Math.PI * kWheelRadius * RobotMap.Swerve.SWERVE_CONVERSION_FACTOR))
        .withAcceleration(10));
    } else {
      driveMotor.setControl(new VoltageOut(0));
 
    }
    if(Math.abs(Units.radiansToRotations(turningMotor.getPosition().getValueAsDouble())-Units.radiansToRotations(normalizeAngle2(desiredState.angle.getRadians())))>5.0/360){
      turningMotor.setControl(positionAngleControl.withPosition(Units.radiansToRotations(normalizeAngle2(desiredState.angle.getRadians()))).withVelocity(40*Units.radiansToRotations(DriveSubsystem.kMaxAngularSpeed)).withUpdateFreqHz(1000));
    } else {
      turningMotor.setControl(new VoltageOut(0));
    }
  }
}