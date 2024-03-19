package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class RobotMap {
	public static final class Swerve {

		/* CAN IDs of Drive Motors */
		public static final int LEFT_FRONT_DRIVE_ID = 2;
		public static final int RIGHT_FRONT_DRIVE_ID = 5;
		public static final int LEFT_BACK_DRIVE_ID = 8;
		public static final int RIGHT_BACK_DRIVE_ID = 11;

		/* Steer Encoder CAN IDs */
		public static final int LEFT_FRONT_STEER_CANCODER_ID = 3;
		public static final int RIGHT_FRONT_STEER_CANCODER_ID = 6;
		public static final int LEFT_BACK_STEER_CANCODER_ID = 9;
		public static final int RIGHT_BACK_STEER_CANCODER_ID = 12;

		/* CAN IDs of steer Motors turning */
		public static final int LEFT_FRONT_STEER_ID = 4;
		public static final int RIGHT_FRONT_STEER_ID = 7;
		public static final int LEFT_BACK_STEER_ID = 10;
		public static final int RIGHT_BACK_STEER_ID = 13;

		 /* Steer Motor Offset */
		 public static final double LEFT_BACK_STEER_OFFSET = -101.33789113045673/360;
		 public static final double RIGHT_BACK_STEER_OFFSET = (-92.68868598068617+1.054687395947221-0.703125072927089+180)/360;
		 public static final double LEFT_FRONT_STEER_OFFSET =(-73.0371115875356+180)/360;
		 public static final double RIGHT_FRONT_STEER_OFFSET = (-81.4924076667695+18.01757799272466+2.197265666134167)/360;
 
		 public static final double SWERVE_CONVERSION_FACTOR = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
 
		 public static final double PHOTON_STDDEV_SCALING_FACTOR = 1.0;

		public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
	}

	public static final class PhotonvisionConstants {
		public static final double FRONT_LEFT_CAMERA_HEIGHT_METERS = 0.171; // NOT FINAL
		public static final double BACK_RIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(8);
		public static final double BACK_LEFT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(9);
		public static final double FRONT_RIGHT_CAMERA_HEIGHT_METERS = 0.146; // NOT IFNAL
		public static final double GRID_TARGET_HEIGHT_METERS = 0.36;
		public static final double DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS = 0.59;
		public static final double BACK_CAMERA_PITCH_RADIANS = Math.toRadians(30);

		// public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA= new
		// Transform3d(new Translation3d(0.381, 0, FRONT_LEFT_CAMERA_HEIGHT_METERS),new
		// Rotation3d(0.0,0.0,Math.toRadians(45.0)));
		public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(
				new Translation3d(-Units.inchesToMeters(8.5), -Units.inchesToMeters(12.5),
						BACK_RIGHT_CAMERA_HEIGHT_METERS),
				new Rotation3d(0, Math.toRadians(30), Math.toRadians(-135)));
		public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(
				new Translation3d(-Units.inchesToMeters(9), Units.inchesToMeters(11), BACK_LEFT_CAMERA_HEIGHT_METERS),
				new Rotation3d(0, Math.toRadians(30), Math.toRadians(135)));
		// public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA= new
		// Transform3d(new Translation3d(0,-0.381,FRONT_RIGHT_CAMERA_HEIGHT_METERS),new
		// Rotation3d(0,0.0,Math.toRadians(315.0)));

	}

	// public static final class DDrive {
	// public static final double STARTING_X = 0;
	// public static final double STARTING_Y = 0;
	// }

	public static final class Shooter {
		public static final int LEFT_ANGLE_MOTOR_ID = 14; // NEED TO CHANGE
		public static final int RIGHT_ANGLE_MOTOR_ID = 15; // NEED TO CHANGE
		public static final int LEFT_FLYWHEEL_MOTOR_ID = 16; // NEED TO CHANGE
		public static final int RIGHT_FLYWHEEL_MOTOR_ID = 17; // NEED TO CHANGE

		public static final double ANGLE_FEED_FORWARD_KS = 0.0001; // NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KG = 0.25; // NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KV = 0.69; // NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KA = 0.00; // NEED TO CHANGE

		public static final double ANGLE_PID_CONTROLLER_P = 5; // NEED TO CHANGE
		public static final double ANGLE_PID_CONTROLLER_I = 0.0; // NEED TO CHANGE
		public static final double ANGLE_PID_CONTROLLER_D = 0.0; // NEED TO CHANGE

		public static final int ANGLE_ENCODER_ID = 6; // wtf is this

		public static final double ANGLE_CONVERSION_FACTOR = 1 / 35.04;

	}

	public static final class Intake {

		public static final int INDEX_MOTOR_ID = 21;
		public static final int BELT_MOTOR_ID = 20;
		public static final int PIVOT_ENCODER_ID = 5;
		public static final int BEAM_PORT = 3;
		public static final int LEFT_PIVOT_MOTOR_ID = 18; // NEED TO CHANGE
		public static final int RIGHT_PIVOT_MOTOR_ID = 19; // NEED TO CHANGE
		// NEED TO CHANGE
		//public static final int PIVOT_ENCODER_ID = 5; // NEED TO CHANGE
		//public static final int HOME_LIMIT_SWITCH_ID = 9999; // NEED TO CHANGE
		// public static final int GROUND_LIMIT_SWITCH_ID = ; // NEED TO CHANGE

		public static final double PIVOT_FEED_FORWARD_KS = 0.0; // NEED TO CHANGE
		public static final double PIVOT_FEED_FORWARD_KG = 7.29; // NEED TO CHANGE
		public static final double PIVOT_FEED_FORWARD_KV = 0.3; // NEED TO CHANGE
		public static final double PIVOT_FEED_FORWARD_KA = 0.29; // NEED TO CHANGE

		public static final double PIVOT_PID_CONTROLLER_P = 2; // NEED TO CHANGE
		public static final double PIVOT_PID_CONTROLLER_I = 0.0; // NEED TO CHANGE
		public static final double PIVOT_PID_CONTROLLER_D = 0.0; // NEED TO CHANGE

		public static final double SIMULATION_OFFSET = 0.0; // NEED TO CHANGE
		public static final double SIMULATION_SCALE = 0.0; // NEED TO CHANGE
		public static final double INTAKE_SIM_LENGTH = 0.0; // NEED TO CHANGE

		public static final double HOME_ANGLE = Math.toRadians(275);
		public static final double GROUND_ANGLE = Math.toRadians(5);// GET ANGLE FROM JOHN

		public static final double GEAR_RATIO = ((double) 1 / 25); // GET FROM SOMEONE
	}

	// public static final class Climer {
	// public static final int RIGHT_CLIMBER_ID = 23;
	// public static final int LEFT_CLIMBER_ID = 24;
	// }

	public static final class FieldConstants {
		public static double fieldLength = Units.inchesToMeters(651.223);
		public static double fieldWidth = Units.inchesToMeters(323.277);
		public static Pose2d centerSpeakOpenInBlue = new Pose2d(0.0, fieldWidth - Units.inchesToMeters(104.0),
				new Rotation2d());

		public static Pose2d centerSpeakOpenInRed = new Pose2d(fieldLength, fieldWidth - Units.inchesToMeters(104.0),
				new Rotation2d());
	}
}
