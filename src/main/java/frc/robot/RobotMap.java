package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class RobotMap {
    public static final class Swerve {

        /* CAN IDs of Drive Motors */
		public static final int LEFT_FRONT_DRIVE_ID = 6;
		public static final int LEFT_BACK_DRIVE_ID = 3;
		public static final int RIGHT_FRONT_DRIVE_ID = 9;
		public static final int RIGHT_BACK_DRIVE_ID = 12;

		/* CAN IDs of steer Motors turning*/
		public static final int LEFT_FRONT_STEER_ID = 4;
		public static final int LEFT_BACK_STEER_ID = 13;
		public static final int RIGHT_FRONT_STEER_ID = 7;
		public static final int RIGHT_BACK_STEER_ID = 10;

		/* Steer Encoder CAN IDs  */
		public static final int LEFT_FRONT_STEER_CANCODER_ID = 5;
		public static final int LEFT_BACK_STEER_CANCODER_ID = 2;
		public static final int RIGHT_FRONT_STEER_CANCODER_ID = 8;
		public static final int RIGHT_BACK_STEER_CANCODER_ID = 11;

        /* Steer Motor Offset */
		public static final double LEFT_FRONT_STEER_OFFSET = -0.1442;
		// public static final double LEFT_FRONT_STEER_OFFSET = Math.toRadians(-19.34);
		public static final double RIGHT_FRONT_STEER_OFFSET = 0.0695;
		public static final double LEFT_BACK_STEER_OFFSET = -0.3110;
		public static final double RIGHT_BACK_STEER_OFFSET = -0.3234;

		public static final double SWERVE_CONVERSION_FACTOR = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);

		public static final double PHOTON_STDDEV_SCALING_FACTOR = (1.0/2.0);

    }

	public static final class PhotonvisionConstants {
		public static final double FRONT_CAMERA_HEIGHT_METERS = 0.171; // NOT FINAL
		public static final double BACK_CAMERA_HEIGHT_METERS = 0.146;
		public static final double RIGHT_CAMERA_HEIGHT_METERS = 0.146; 
		public static final double LEFT_CAMERA_HEIGHT_METERS = 0.146;
		public static final double GRID_TARGET_HEIGHT_METERS = 0.36;
		public static final double DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS = 0.59;
		public static final double CAMERA_PITCH_RADIANS = 0; // NOT FINAL
		
		public static final Transform3d ROBOT_TO_FRONT_CAMERA= new Transform3d(new Translation3d(0.381, 0, FRONT_CAMERA_HEIGHT_METERS),new Rotation3d(0.0,0.0,0.0));
		public static final Transform3d ROBOT_TO_BACK_CAMERA= new Transform3d(new Translation3d(-0.381,0,BACK_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(180),0));
		public static final Transform3d ROBOT_TO_LEFT_CAMERA= new Transform3d(new Translation3d(0,0.381,LEFT_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(90),0));
		public static final Transform3d ROBOT_TO_RIGHT_CAMERA= new Transform3d(new Translation3d(0,-0.381,RIGHT_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(270),0));

	}
	public static final class DDrive {
		public static final double STARTING_X=0;
		public static final double STARTING_Y=0;
	}
}
