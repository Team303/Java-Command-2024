// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.9; // 3.9 meters per second
  public static final double kMaxAngularSpeed = kMaxSpeed / (Math.hypot(0.381, 0.381)); // radians per second

  private final Translation2d frontLeftLocation = new Translation2d(-0.381, -0.381);
  private final Translation2d frontRightLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backLeftLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backRightLocation = new Translation2d(0.381, 0.381);

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveOdometry odometry;
  private final PIDController m_driftCorrectionPid = new PIDController(0.1, 0, 0);
  private Pose2d pose = new Pose2d(0.0, 0.0, new Rotation2d()); 

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds relativeSpeeds = new ChassisSpeeds(); 

  public static final ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Drive Base");

  public static final GenericEntry FRONT_LEFT_ENC = DRIVEBASE_TAB.add("front left enc", 0).withPosition(0, 0).withSize(2, 1).getEntry();
	public static final GenericEntry FRONT_RIGHT_ENC = DRIVEBASE_TAB.add("front right enc", 0).withPosition(2, 0).withSize(2,1).getEntry();
	public static final GenericEntry BACK_LEFT_ENC = DRIVEBASE_TAB.add("back left enc", 0).withPosition(5, 0).withSize(2,1).getEntry();
	public static final GenericEntry BACK_RIGHT_ENC = DRIVEBASE_TAB.add("back right enc", 0).withPosition(7, 0).withSize(2,1).getEntry();

  public static final GenericEntry backRightDriveOutput = DRIVEBASE_TAB.add("back right drive ouptut", 0).withPosition(7,1).withSize(2,1).getEntry();
  public static final GenericEntry backLeftDriveOutput = DRIVEBASE_TAB.add("back left drive ouptut", 0).withPosition(5, 1).withSize(2,1).getEntry();
  public static final GenericEntry frontLeftDriveOutput = DRIVEBASE_TAB.add("front left drive ouptut", 0).withPosition(0, 1).withSize(2,1).getEntry();
  public static final GenericEntry frontRightDriveOutput = DRIVEBASE_TAB.add("front right drive ouptut", 0).withPosition(2, 1).withSize(2,1).getEntry();

  public static final GenericEntry backRightTurnOutput = DRIVEBASE_TAB.add("back right turn ouptut", 0).withPosition(7, 2).withSize(2,1).getEntry();
  public static final GenericEntry backLeftTurnOutput = DRIVEBASE_TAB.add("back left turn ouptut", 0).withPosition(5, 2).withSize(2,1).getEntry();
  public static final GenericEntry frontLeftTurnOutput = DRIVEBASE_TAB.add("front left turn ouptut", 0).withPosition(0, 2).withSize(2,1).getEntry();
  public static final GenericEntry frontRightTurnOutput = DRIVEBASE_TAB.add("front right turn ouptut", 0).withPosition(2, 2).withSize(2,1).getEntry();

  public static final GenericEntry backRightAngle = DRIVEBASE_TAB.add("back right angle", 0).withPosition(7, 3).withSize(2,1).getEntry();
  public static final GenericEntry backLeftAngle = DRIVEBASE_TAB.add("back left angle", 0).withPosition(5, 3).withSize(2,1).getEntry();
  public static final GenericEntry frontLeftAngle = DRIVEBASE_TAB.add("front left angle", 0).withPosition(0, 3).withSize(2,1).getEntry();
  public static final GenericEntry frontRightAngle = DRIVEBASE_TAB.add("front right angle", 0).withPosition(2, 3).withSize(2,1).getEntry();

  public static final GenericEntry globalAngle = DRIVEBASE_TAB.add("global angle", 0).withPosition(4, 0).getEntry();
  public static final GenericEntry angleVelo = DRIVEBASE_TAB.add("angular velocity", 0).withPosition(4,1).getEntry();
  public static final GenericEntry time = DRIVEBASE_TAB.add("Time", 0).withPosition(4, 2).getEntry();

  public static final GenericEntry translationalVelo = DRIVEBASE_TAB.add("transational velocity", 0).withPosition(4,3).getEntry();

  public static double angularVelocity = 0;

  private double m_desiredHeading = 0;

  private final SwerveDriveKinematics kinematics =
    new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final Timer AVTimer = new Timer();

  public Drivetrain() {
    Robot.navX.reset();
    AVTimer.start();

    frontLeft = new SwerveModule(
      RobotMap.Swerve.LEFT_FRONT_DRIVE_ID, 
      RobotMap.Swerve.LEFT_FRONT_STEER_ID, 
      RobotMap.Swerve.LEFT_FRONT_STEER_CANCODER_ID
      );
    frontRight = new SwerveModule(
      RobotMap.Swerve.RIGHT_FRONT_DRIVE_ID, 
      RobotMap.Swerve.RIGHT_FRONT_STEER_ID, 
      RobotMap.Swerve.RIGHT_FRONT_STEER_CANCODER_ID
      );
    backLeft = new SwerveModule(
      RobotMap.Swerve.LEFT_BACK_DRIVE_ID, 
      RobotMap.Swerve.LEFT_BACK_STEER_ID, 
      RobotMap.Swerve.LEFT_BACK_STEER_CANCODER_ID
      );
    backRight = new SwerveModule(
      RobotMap.Swerve.RIGHT_BACK_DRIVE_ID, 
      RobotMap.Swerve.RIGHT_BACK_STEER_ID, 
      RobotMap.Swerve.RIGHT_BACK_STEER_CANCODER_ID
    );


    frontLeft.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.LEFT_FRONT_STEER_OFFSET);
    frontRight.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.RIGHT_FRONT_STEER_OFFSET);
    backLeft.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.LEFT_BACK_STEER_OFFSET);
    backRight.getTurningEncoder().configMagnetOffset(RobotMap.Swerve.RIGHT_BACK_STEER_OFFSET);

    frontLeft.invertSteerMotor(true);
    frontRight.invertSteerMotor(true);  
    backRight.invertSteerMotor(true);

    frontLeft.invertDriveMotor(true);
    frontRight.invertDriveMotor(true);
    backRight.invertDriveMotor(true);

    odometry = new SwerveDriveOdometry(
      kinematics,
      Robot.navX.getRotation2d(),
      getModulePositions()
    );

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(8, 0, 0), // Rotation PID constants
                    3.9, // Max module speed, in m/s
                    0.381, // Drive base radius in meters. Distance from robot center to furthest module.
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
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Robot.navX.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
    // swerveModuleStates = kinematics.toSwerveModuleStates(translationalDriftCorrection(kinematics.toChassisSpeeds(swerveModuleStates)));

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
    if(!Robot.navX.isConnected())
      return chassisSpeeds;
    double translationalVelocity = Math.abs(frontLeft.getDriveVelocity());
    Logger.recordOutput("translational velocity", translationalVelocity); 

    if (Math.abs(Robot.navX.getRate()) > 0.3) {
      m_desiredHeading = Robot.navX.getYaw();
    } else if (translationalVelocity > 1) {

      double calc = m_driftCorrectionPid.calculate(Robot.navX.getYaw(),
          m_desiredHeading);

      if (Math.abs(calc) >= 0.55) {
        chassisSpeeds.omegaRadiansPerSecond += calc;
      }
    }
    return chassisSpeeds;
  }

  private ChassisSpeeds rotationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
  // Assuming the control loop runs in 20ms
  final double deltaTime = 0.02;

  // The position of the bot one control loop in the future given the chassisspeed
  Pose2d robotPoseVel = new Pose2d(chassisSpeeds.vxMetersPerSecond * deltaTime,
      chassisSpeeds.vyMetersPerSecond * deltaTime,
      new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * deltaTime));

  Twist2d twistVel = new Pose2d(0, 0, new Rotation2d()).log(robotPoseVel);
  return new ChassisSpeeds(
      twistVel.dx / deltaTime, twistVel.dy / deltaTime,
      twistVel.dtheta / deltaTime);
  }

  public void drive(SwerveModuleState[] state) {

    frontLeftAngle.setDouble(state[0].angle.getDegrees());
    frontRightAngle.setDouble(state[1].angle.getDegrees());
    backLeftAngle.setDouble(state[2].angle.getDegrees());
    backRightAngle.setDouble(state[3].angle.getDegrees());

    frontLeft.setDesiredState(state[0]);
    frontRight.setDesiredState(state[1]);
    backLeft.setDesiredState(state[2]);
    backRight.setDesiredState(state[3]);
    
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {

    relativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    if (fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(translation.getX(), translation.getY(), rotation), Rotation2d.fromDegrees(-Robot.navX.getAngle()));
    } else {
      chassisSpeeds = relativeSpeeds;
    }

    chassisSpeeds = translationalDriftCorrection(chassisSpeeds);
    
    // chassisSpeeds = rotationalDriftCorrection(chassisSpeeds);
    
    drive(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void robotRelativeDrive(ChassisSpeeds chassisSpeeds) {

    chassisSpeeds = translationalDriftCorrection(chassisSpeeds);

    // chassisSpeeds = rotationalDriftCorrection(chassisSpeeds);

    drive(kinematics.toSwerveModuleStates(chassisSpeeds));

    // SwerveModuleState[] state = kinematics.toSwerveModuleStates(chassisSpeeds);

    // frontLeftAngle.setDouble(state[0].angle.getDegrees());
    // frontRightAngle.setDouble(state[1].angle.getDegrees());
    // backLeftAngle.setDouble(state[2].angle.getDegrees());
    // backRightAngle.setDouble(state[3].angle.getDegrees());

    // frontLeft.setDesiredState(state[0]);
    // frontRight.setDesiredState(state[1]);
    // backLeft.setDesiredState(state[2]);
    // backRight.setDesiredState(state[3]);
    // System.out.println("Should be driving!");
  }

  public Command followPathFromFile(String pathToFile) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathToFile);
    return AutoBuilder.followPath(path);
  }

  public Command getAutonomousCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    pose = odometry.update(
        Robot.navX.getRotation2d(),
        getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public void resetOdometry() {
    Robot.navX.reset();
    odometry.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), pose);
  }

  public void resetPose(Pose2d pose) {
    Robot.navX.reset();
    odometry.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), new Pose2d());
  }

  public Pose2d getPose() {
    return pose; 
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void periodic() {

    FRONT_LEFT_ENC.setDouble(frontLeft.getPosition().angle.getDegrees());
		FRONT_RIGHT_ENC.setDouble(frontRight.getPosition().angle.getDegrees());
		BACK_LEFT_ENC.setDouble(backLeft.getPosition().angle.getDegrees());
		BACK_RIGHT_ENC.setDouble(backRight.getPosition().angle.getDegrees());

    frontLeftDriveOutput.setDouble(frontLeft.getMainDriveOutput());
    backLeftDriveOutput.setDouble(backLeft.getMainDriveOutput());
    frontRightDriveOutput.setDouble(frontRight.getMainDriveOutput());
    backRightDriveOutput.setDouble(backRight.getMainDriveOutput());

    frontLeftTurnOutput.setDouble(frontLeft.getMainTurnOutput());
    backLeftTurnOutput.setDouble(backLeft.getMainTurnOutput());
    frontRightTurnOutput.setDouble(frontRight.getMainTurnOutput());
    backRightTurnOutput.setDouble(backRight.getMainTurnOutput());

    globalAngle.setDouble(Robot.navX.getAngle()); 
    angleVelo.setDouble(Robot.navX.getRate());
    time.setDouble(AVTimer.get());

    updateOdometry();
    Logger.recordOutput("Odometry", pose);
    Logger.recordOutput("angular velocity", Robot.navX.getRate());
    Logger.recordOutput("FrontLeft Voltage", frontLeft.getDriveCurrent());
    Logger.recordOutput("FrontRight Voltage", frontRight.getDriveCurrent());
    Logger.recordOutput("BackLeft Voltage", backLeft.getDriveCurrent());
    Logger.recordOutput("BackRight Voltage", backRight.getDriveCurrent());


  }
}



