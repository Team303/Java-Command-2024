package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class RobotMap {
	public static final class Swerve {


		//This should stay pretty similar but may change slightly
		/* CAN IDs of Drive Motors */
		public static final int LEFT_FRONT_DRIVE_ID = 2;
		public static final int RIGHT_FRONT_DRIVE_ID = 8;
		public static final int LEFT_BACK_DRIVE_ID = 5;
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
		public static final double LEFT_BACK_STEER_OFFSET = ( -101.33789113045673 - 60.90) / 360;
		public static final double RIGHT_BACK_STEER_OFFSET = (180-83.4 + 11.32359884120018 -92.68868598068617 + 2.050905483056468 + 9.03457887096521 + 1.054687395947221 - 0.703125072927089
				+ 180) / 360;
		public static final double LEFT_FRONT_STEER_OFFSET = (180+148.71 + 11.32359884120018 + -73.0371115875356 + 180) / 360;
		public static final double RIGHT_FRONT_STEER_OFFSET = ( 15 -39.023 + 81.4924076667695 + 18.01757799272466
				+ 2.197265666134167) / 360;

		public static final double SWERVE_CONVERSION_FACTOR = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

		public static final double PHOTON_STDDEV_SCALING_FACTOR = 1.0;

		public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
	}

	//TODO: Figure this out based off of the final design
	public static final class PhotonvisionConstants {
		public static final double FRONT_LEFT_CAMERA_HEIGHT_METERS = 0.25;
		// public static final double BACK_RIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(8);
		// public static final double BACK_LEFT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(9);
		public static final double FRONT_RIGHT_CAMERA_HEIGHT_METERS = 0.25; // NOT IFNAL
		public static final double GRID_TARGET_HEIGHT_METERS = 0.36;
		public static final double DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS = 0.59;
		public static final double BACK_CAMERA_PITCH_RADIANS = Math.toRadians(30);

		// public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA= new
		// Transform3d(new Translation3d(0.381, 0, FRONT_LEFT_CAMERA_HEIGHT_METERS),new
		// Rotation3d(0.0,0.0,Math.toRadians(45.0)));
		public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
				new Translation3d(-Units.inchesToMeters(9.375), Units.inchesToMeters(13.5),
						FRONT_LEFT_CAMERA_HEIGHT_METERS),
				new Rotation3d(180, Math.toRadians(18), Math.toRadians(90)));
		public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
				new Translation3d(Units.inchesToMeters(10.5), Units.inchesToMeters(13.5), FRONT_RIGHT_CAMERA_HEIGHT_METERS),
				new Rotation3d(Math.toRadians(180), Math.toRadians(18), Math.toRadians(90)));
		// public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA= new
		// Transform3d(new Translation3d(0,-0.381,FRONT_RIGHT_CAMERA_HEIGHT_METERS),new
		// Rotation3d(0,0.0,Math.toRadians(315.0)));

	}
	//Subteam-specific constants go here
 
	

	//Field-specific constants go here
	//TODO: Change field-specific constants
	public static final class FieldConstants {
		public static double fieldLength = Units.inchesToMeters(684.223);
		public static double fieldWidth = Units.inchesToMeters(317);
		public static Pose2d centerSpeakOpenInBlue = new Pose2d(0.0, Units.inchesToMeters(323.277) - Units.inchesToMeters(104.0),
				new Rotation2d());

		public static Pose2d centerSpeakOpenInRed = new Pose2d(fieldLength, fieldWidth - Units.inchesToMeters(104.0),
				new Rotation2d());
	}
}
