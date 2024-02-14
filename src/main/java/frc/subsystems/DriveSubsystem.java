// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SwerveModule;
import frc.robot.RobotMap.Swerve;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.RobotMap.Swerve;

import frc.robot.SwerveModule;
import frc.modules.PhotonvisionModule.CameraName;
import frc.robot.Robot;
import frc.robot.RobotMap;
// import frc.robot.RobotMap.PhotonvisionConstants;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 5.2; // 5.2 meters per second
  public static final double kMaxAngularSpeed = kMaxSpeed / (Math.hypot(0.3302, 0.3302)); // radians per second

  private final Translation2d frontLeftLocation = new Translation2d(0.3302, 0.3302);
  private final Translation2d frontRightLocation = new Translation2d(0.3302, -0.3302);
  private final Translation2d backLeftLocation = new Translation2d(-0.3302, 0.3302);
  private final Translation2d backRightLocation = new Translation2d(-0.3302, -0.3302);

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;
  int jump=0;
  // private final SwerveDriveOdometry odometry;
  private final PIDController m_driftCorrectionPid = new PIDController(0.12, 0, 0);
  // private Pose2d pose = new Pose2d(0.0, 0.0, new Rotation2d());

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds relativeSpeeds = new ChassisSpeeds(); 

  public static final ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Drive Base");

  public static final GenericEntry FRONT_LEFT_ENC = DRIVEBASE_TAB.add("front left enc", 0).withPosition(0, 0)
      .withSize(2, 1).getEntry();
  public static final GenericEntry FRONT_RIGHT_ENC = DRIVEBASE_TAB.add("front right enc", 0).withPosition(2, 0)
      .withSize(2, 1).getEntry();
  public static final GenericEntry BACK_LEFT_ENC = DRIVEBASE_TAB.add("back left enc", 0).withPosition(5, 0)
      .withSize(2, 1).getEntry();
  public static final GenericEntry BACK_RIGHT_ENC = DRIVEBASE_TAB.add("back right enc", 0).withPosition(7, 0)
      .withSize(2, 1).getEntry();

  public static final GenericEntry backRightDriveEncoder = DRIVEBASE_TAB.add("back right drive enc", 0)
      .withPosition(7, 1).withSize(2, 1).getEntry();
  public static final GenericEntry backLeftDriveEncoder = DRIVEBASE_TAB.add("back left drive enc", 0)
      .withPosition(5, 1).withSize(2, 1).getEntry();
  public static final GenericEntry frontLeftDriveEncoder = DRIVEBASE_TAB.add("front left drive enc", 0)
      .withPosition(0, 1).withSize(2, 1).getEntry();
  public static final GenericEntry frontRightDriveEncoder = DRIVEBASE_TAB.add("front right drive enc", 0)
      .withPosition(2, 1).withSize(2, 1).getEntry();

  public static final GenericEntry backRightTurnOutput = DRIVEBASE_TAB.add("back right turn ouptut", 0)
      .withPosition(7, 2).withSize(2, 1).getEntry();
  public static final GenericEntry backLeftTurnOutput = DRIVEBASE_TAB.add("back left turn ouptut", 0).withPosition(5, 2)
      .withSize(2, 1).getEntry();
  public static final GenericEntry frontLeftTurnOutput = DRIVEBASE_TAB.add("front left turn ouptut", 0)
      .withPosition(0, 2).withSize(2, 1).getEntry();
  public static final GenericEntry frontRightTurnOutput = DRIVEBASE_TAB.add("front right turn ouptut", 0)
      .withPosition(2, 2).withSize(2, 1).getEntry();

  public static final GenericEntry backRightAngle = DRIVEBASE_TAB.add("back right angle", 0).withPosition(7, 3)
      .withSize(2, 1).getEntry();
  public static final GenericEntry backLeftAngle = DRIVEBASE_TAB.add("back left angle", 0).withPosition(5, 3)
      .withSize(2, 1).getEntry();
  public static final GenericEntry frontLeftAngle = DRIVEBASE_TAB.add("front left angle", 0).withPosition(0, 3)
      .withSize(2, 1).getEntry();
  public static final GenericEntry frontRightAngle = DRIVEBASE_TAB.add("front right angle", 0).withPosition(2, 3)
      .withSize(2, 1).getEntry();

  public static final GenericEntry resetPoseX = DRIVEBASE_TAB.add("resetPoseX", 0).withPosition(0, 4)
      .withSize(1, 1).getEntry();
  public static final GenericEntry resetPoseY = DRIVEBASE_TAB.add("resetPoseY", 0).withPosition(1, 4)
    .withSize(1, 1).getEntry();
  public static final GenericEntry resetPoseAngle = DRIVEBASE_TAB.add("resetPoseAngle", 0).withPosition(2, 4)
    .withSize(1, 1).getEntry();
  
  public static final GenericEntry globalAngle = DRIVEBASE_TAB.add("global angle", 0).withPosition(4, 0).getEntry();
  public static final GenericEntry angleVelo = DRIVEBASE_TAB.add("angular velocity", 0).withPosition(4, 1).getEntry();
  // public static final GenericEntry time = DRIVEBASE_TAB.add("Time",
  // 0).withPosition(4, 2).getEntry();

  public static final GenericEntry translationalVelo = DRIVEBASE_TAB.add("transational velocity", 0).withPosition(4, 3)
      .getEntry();

  public static final GenericEntry angleToSpeaker = DRIVEBASE_TAB.add("angleToSpeaker", 0).withPosition(3, 4)
      .getEntry();

  public static double angularVelocity = 0;

  private double m_desiredHeading = 0;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final Timer AVTimer = new Timer();

  // public final AprilTagFieldLayout aprilTagField;
  private final Field2d field2d = new Field2d();
  private static final Vector<N3> odometryStandardDeviations = VecBuilder.fill(5, 5, Units.degreesToRadians(10));
  // private static final Vector<N3> photonStandardDeviations =
  // VecBuilder.fill(0.25, 0.25, 0);
  private static final Vector<N3> photonStandardDeviations = VecBuilder.fill(5, 5, 100);
  private static final Vector<N3> kSingleStandardDeviations = VecBuilder.fill(5,5,100);
  private static final Vector<N3> kMultiTagStandardDeviations = VecBuilder.fill(2.5,2.5,100);


  public CANcoderConfiguration configLeftFront;
  public CANcoderConfiguration configRightFront;
  public CANcoderConfiguration configLeftBack;  
  public CANcoderConfiguration configRightBack;

  public SwerveDrivePoseEstimator poseEstimator;

  public DriveSubsystem() {

    Logger.recordOutput("Swerve Module States", new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()});
    Robot.navX.reset();
    AVTimer.start();

    configLeftFront = new CANcoderConfiguration();
    configLeftFront.MagnetSensor.MagnetOffset = Swerve.LEFT_FRONT_STEER_OFFSET;
    configRightFront = new CANcoderConfiguration();
    configRightFront.MagnetSensor.MagnetOffset = Swerve.RIGHT_FRONT_STEER_OFFSET;
    configLeftBack = new CANcoderConfiguration();
    configLeftBack.MagnetSensor.MagnetOffset = Swerve.LEFT_BACK_STEER_OFFSET;
    configRightBack = new CANcoderConfiguration();
    configRightBack.MagnetSensor.MagnetOffset = Swerve.RIGHT_BACK_STEER_OFFSET;

    PIDController frontLeftPID = new PIDController(6, 0.5, 0);
    frontLeftPID.setIntegratorRange(0,1);

    PIDController frontRightPID = new PIDController(6, 0.5, 0);
    frontLeftPID.setIntegratorRange(0,1);

    PIDController backLeftPID = new PIDController(6, 0.5, 0);
    frontLeftPID.setIntegratorRange(0,1);

    PIDController backRightPID = new PIDController(6, 0.5, 0);
    frontLeftPID.setIntegratorRange(0,1);



    frontLeft = new SwerveModule(
      RobotMap.Swerve.LEFT_FRONT_DRIVE_ID, 
      RobotMap.Swerve.LEFT_FRONT_STEER_ID,
      RobotMap.Swerve.LEFT_FRONT_STEER_CANCODER_ID,
      configLeftFront
      );

    frontRight = new SwerveModule(
      RobotMap.Swerve.RIGHT_FRONT_DRIVE_ID, 
      RobotMap.Swerve.RIGHT_FRONT_STEER_ID,
      RobotMap.Swerve.RIGHT_FRONT_STEER_CANCODER_ID,
      configRightFront
      );
    backLeft = new SwerveModule(
      RobotMap.Swerve.LEFT_BACK_DRIVE_ID, 
      RobotMap.Swerve.LEFT_BACK_STEER_ID,
      RobotMap.Swerve.LEFT_BACK_STEER_CANCODER_ID,
      configLeftBack
    );
    backRight = new SwerveModule(
      RobotMap.Swerve.RIGHT_BACK_DRIVE_ID, 
      RobotMap.Swerve.RIGHT_BACK_STEER_ID,
      RobotMap.Swerve.RIGHT_BACK_STEER_CANCODER_ID,
      configRightBack
    );

    // frontLeft.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.LEFT_FRONT_STEER_OFFSET);
    // frontRight.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.RIGHT_FRONT_STEER_OFFSET);
    // backLeft.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.LEFT_BACK_STEER_OFFSET);
    // backRight.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.RIGHT_BACK_STEER_OFFSET);

    frontLeft.invertSteerMotor(true);
    frontRight.invertSteerMotor(true);
    backRight.invertSteerMotor(true);
    backLeft.invertSteerMotor(true);

    frontLeft.invertDriveMotor(true);
    backLeft.invertDriveMotor(true);
    frontRight.invertDriveMotor(false);
    backRight.invertDriveMotor(false);

    AprilTagFieldLayout initialLayout;
    // try {
      
    //   initialLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    //   Optional<Alliance> alliance = DriverStation.getAlliance();
    //   // TODO: Change to make the origin position based off of station rather than
    //   // just based off of alliance.
    //   initialLayout
    //       .setOrigin(alliance.isPresent() && alliance.get() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
    //           : OriginPosition.kRedAllianceWallRightSide);
    // } catch (IOException e) {
    //   DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    //   initialLayout = null;
    // }
    // aprilTagField = initialLayout;
    // if (Robot.isReal()) {
    //   visionPoseEstimatorFront = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //       Robot.photonvision.getCamera(CameraName.CAM1),
    //       PhotonvisionConstants.ROBOT_TO_FRONT_CAMERA);
    //   // visionPoseEstimatorRight = new PhotonPoseEstimator(aprilTagField,
    //   // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //   // Robot.photonvision.getCamera(CameraName.CAM2),
    //   // PhotonvisionConstants.ROBOT_TO_RIGHT_CAMERA);
    //   visionPoseEstimatorBack = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //       Robot.photonvision.getCamera(CameraName.CAM3),
    //       PhotonvisionConstants.ROBOT_TO_BACK_CAMERA);
    //   // visionPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagField,
    //   // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //   // Robot.photonvision.getCamera(CameraName.CAM4),
    //   // PhotonvisionConstants.ROBOT_TO_LEFT_CAMERA);
    //   visionPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    //   // visionPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    //   visionPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    //   // visionPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // }

    // odometry = new SwerveDriveOdometry(
    // kinematics,
    // Robot.navX.getRotation2d(),
    // getModulePositions());
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, Robot.navX.getRotation2d(), getModulePositions(),
        new Pose2d(new Translation2d(), new Rotation2d()),
        odometryStandardDeviations, photonStandardDeviations);

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(8, 0, 0), // Rotation PID constants
                    5.2, // Max module speed, in m/s
                    0.3302, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Robot.navX.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // swerveModuleStates =
    // kinematics.toSwerveModuleStates(translationalDriftCorrection(kinematics.toChassisSpeeds(swerveModuleStates)));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Adds rotational velocity to the chassis speed to compensate for
   * unwanted changes in gyroscope heading.
   * 
   * @param chassisSpeeds the given chassisspeeds
   * @return the corrected chassisspeeds
   */
  private ChassisSpeeds translationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
    if (!Robot.navX.isConnected())
      return chassisSpeeds;
    double translationalVelocity = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Logger.recordOutput("translational velocity", translationalVelocity);
    Logger.recordOutput("turn rate",Robot.navX.getRate());

    if (Math.abs(Robot.navX.getRate()) > 0.5) {
      m_desiredHeading = Robot.navX.getYaw();
    } else if (Math.abs(translationalVelocity) > 1) {

      double calc = m_driftCorrectionPid.calculate(Robot.navX.getYaw(),
          m_desiredHeading);

      if (Math.abs(calc) >= 0.1) 
      {
        chassisSpeeds.omegaRadiansPerSecond -= calc;
      }
    }
    return chassisSpeeds;
  }

  public void drive(SwerveModuleState[] state) {

    // frontLeftAngle.setDouble(state[0].angle.getDegrees());
    // frontRightAngle.setDouble(state[1].angle.getDegrees());
    // backLeftAngle.setDouble(state[2].angle.getDegrees());
    // backRightAngle.setDouble(state[3].angle.getDegrees());

    Logger.recordOutput("Swerve Module States", state);

    frontLeft.setDesiredState(state[0]);
    frontRight.setDesiredState(state[1]);
    backLeft.setDesiredState(state[2]);
    backRight.setDesiredState(state[3]);

    System.out.println("Driving");
    
  }
 
  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
        Rotation2d.fromDegrees(-Robot.navX.getAngle()));

    chassisSpeeds = translationalDriftCorrection(chassisSpeeds);

    Logger.recordOutput("Swerve Module States", kinematics.toSwerveModuleStates(chassisSpeeds));

    drive(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void robotRelativeDrive(ChassisSpeeds chassisSpeeds) {

    // chassisSpeeds = translationalDriftCorrection(chassisSpeeds);
    // double vx = chassisSpeeds.vxMetersPerSecond;
    // chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vyMetersPerSecond;
    // chassisSpeeds.vyMetersPerSecond = vx;

    drive(kinematics.toSwerveModuleStates(chassisSpeeds));

  }

  public Command followPathFromFile(String pathToFile) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathToFile);
    return AutoBuilder.followPath(path);
  }

  public Command getAutonomousCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry(boolean jumpFront, boolean jumpBack) {
    poseEstimator.update(Robot.navX.getRotation2d(), getModulePositions());
    
    field2d.setRobotPose(getPose());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry() {

    boolean isAlliance = true;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        isAlliance = alliance.get() == DriverStation.Alliance.Blue;
    }

    if (isAlliance) 
      Robot.navX.setAngleAdjustment(0);
    else
      Robot.navX.setAngleAdjustment(180);
    Robot.navX.reset();

    poseEstimator.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), new Pose2d(new Translation2d(), Rotation2d.fromDegrees(Robot.navX.getAngle())));
  }

  public void resetOdometry(Pose2d pose) {
    Robot.navX.setAngleAdjustment(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), pose);
    // odometry.resetPosition(Robot.navX.getRotation2d(), getModulePositions(),
    // pose);
  }
  
  public void resetOdometryWidget() {

    boolean isAlliance = true;
    Rotation2d angleAdjustment;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        isAlliance = alliance.get() == DriverStation.Alliance.Blue;
    }

    if (isAlliance) 
      angleAdjustment = Rotation2d.fromDegrees(0);
    else
      angleAdjustment = Rotation2d.fromDegrees(180);


    resetOdometry(new Pose2d(new Translation2d(resetPoseX.getDouble(0), resetPoseY.getDouble(0)), angleAdjustment));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

  public double calculateAngleSpeaker() {
    boolean isAlliance = true;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        isAlliance = alliance.get() == DriverStation.Alliance.Blue;
    }

    Pose2d robotPose = getPose();
    Translation2d speakerPose;

    speakerPose = isAlliance ? new Translation2d(0.5, 5.5) :new Translation2d(16.2, 5.5);

    return -Math.atan2(speakerPose.getY() - robotPose.getY(), speakerPose.getX() - robotPose.getX()) * (180 / Math.PI);

  }
 

  @Override
  public void periodic() {

    FRONT_LEFT_ENC.setDouble(frontLeft.turningEncoder.getAbsolutePosition().refresh().getValue());
    FRONT_RIGHT_ENC.setDouble(frontRight.turningEncoder.getAbsolutePosition().refresh().getValue());
    BACK_LEFT_ENC.setDouble(backLeft.turningEncoder.getAbsolutePosition().refresh().getValue());
    BACK_RIGHT_ENC.setDouble(backRight.turningEncoder.getAbsolutePosition().refresh().getValue());

    frontLeftDriveEncoder.setDouble(frontLeft.getPosition().distanceMeters);
    backLeftDriveEncoder.setDouble(backLeft.getPosition().distanceMeters);
    frontRightDriveEncoder.setDouble(frontRight.getPosition().distanceMeters);
    backRightDriveEncoder.setDouble(backRight.getPosition().distanceMeters);

    frontLeftTurnOutput.setDouble(frontLeft.getMainTurnOutput());
    backLeftTurnOutput.setDouble(backLeft.getMainTurnOutput());
    frontRightTurnOutput.setDouble(frontRight.getMainTurnOutput());
    backRightTurnOutput.setDouble(backRight.getMainTurnOutput());
    angleToSpeaker.setDouble(calculateAngleSpeaker());

    globalAngle.setDouble(Robot.navX.getAngle() % 360);
    angleVelo.setDouble(Robot.navX.getRate());
    // if(jump<100){
      updateOdometry(true,true);
      // jump++;
    // } else {
    //   updateOdometry(false,false);
    // }

    Logger.recordOutput("Odometry", getPose());
    Logger.recordOutput("angular velocity", Robot.navX.getRate());
    Logger.recordOutput("Front Left Arya", frontLeft.getPosition().angle.getDegrees());
    Logger.recordOutput("Back Left Arav", backLeft.getPosition().angle.getDegrees());
    Logger.recordOutput("Front Right Alan", frontRight.getPosition().angle.getDegrees());
    Logger.recordOutput("Back Right Aritra", backRight.getPosition().angle.getDegrees());
    Logger.recordOutput("Alan is a persecuter", true);
    Logger.recordOutput("Real Swerve Module States", getModuleStates());
    

    // Logger.recordOuptu("Swervemodule states", Swer)
  }
}
