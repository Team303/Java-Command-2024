package frc.robot;

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
		public static final double LEFT_FRONT_STEER_OFFSET = -22.62 + 65;
		// public static final double LEFT_FRONT_STEER_OFFSET = Math.toRadians(-19.34);
		public static final double RIGHT_FRONT_STEER_OFFSET = -68.5;
		public static final double LEFT_BACK_STEER_OFFSET = -23.5;
		public static final double RIGHT_BACK_STEER_OFFSET = -208.3;

		public static final double SWERVE_CONVERSION_FACTOR = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0); 

    }

	public static final class Shooter {
		public static final int LEFT_ANGLE_MOTOR_ID = 0; //NEED TO CHANGE
		public static final int RIGHT_ANGLE_MOTOR_ID = 1; //NEED TO CHANGE
		public static final int LEFT_FLYWHEEL_MOTOR_ID = 2; //NEED TO CHANGE
		public static final int RIGHT_FLYWHEEL_MOTOR_ID = 3; //NEED TO CHANGE
		public static final int LEFT_INDEXER_MOTOR_ID = 4; //NEED TO CHANGE
		public static final int RIGHT_INDEXER_MOTOR_ID = 5; //NEED TO CHANGE

		public static final double ANGLE_FEED_FORWARD_KS = 0.0; //NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KG = 0.0; //NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KV = 0.0; //NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KA = 0.0; //NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_VEL = 2.0; //NEED TO TWEAK

		public static final double FLYWHEEL_FEED_FORWARD_KS = 0.0; //NEED TO CHANGE
		public static final double FLYWHEEL_FEED_FORWARD_KV = 0.0; //NEED TO CHANGE
		public static final double FLYWHEEL_FEED_FORWARD_KA = 0.0; //NEED TO CHANGE

		public static final double ANGLE_PID_CONTROLLER_P = 0.0; //NEED TO CHANGE
		public static final double ANGLE_PID_CONTROLLER_I = 0.0; //NEED TO CHANGE
		public static final double ANGLE_PID_CONTROLLER_D = 0.0; //NEED TO CHANGE

		public static final int BEAM_BREAK_ID = 0;
	}

	public static final class Intake {
		public static final int LEFT_BOTTOM_MOTOR_ID = 0; //NEED TO CHANGE
		public static final int RIGHT_BOTTOM_MOTOR_ID = 1; //NEED TO CHANGE
		public static final int LEFT_TOP_MOTOR_ID = 2; //NEED TO CHANGE
		public static final int RIGHT_TOP_MOTOR_ID = 3; //NEED TO CHANGE
		public static final int INTAKE_ROLLER_MOTOR_ID = 4; //NEED TO CHANGE


	}
}
