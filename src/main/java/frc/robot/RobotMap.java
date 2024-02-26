package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Nat;

public class RobotMap {
	public static final class Swerve {

		/* CAN IDs of Drive Motors */
		 public static final int LEFT_FRONT_DRIVE_ID = 4;
		 public static final int LEFT_BACK_DRIVE_ID = 6;
		 public static final int RIGHT_FRONT_DRIVE_ID = 10;
		 public static final int RIGHT_BACK_DRIVE_ID = 13;
 
		 /* CAN IDs of steer Motors turning */
		 public static final int LEFT_FRONT_STEER_ID = 7;
		 public static final int LEFT_BACK_STEER_ID = 9;
		 public static final int RIGHT_FRONT_STEER_ID = 11;
		 public static final int RIGHT_BACK_STEER_ID = 3;
 
		 /* Steer Encoder CAN IDs */
		 public static final int LEFT_FRONT_STEER_CANCODER_ID = 2;
		 public static final int LEFT_BACK_STEER_CANCODER_ID = 5;
		 public static final int RIGHT_FRONT_STEER_CANCODER_ID = 12;
		 public static final int RIGHT_BACK_STEER_CANCODER_ID = 8;
 
		 /* Steer Motor Offset */
		 public static final double LEFT_BACK_STEER_OFFSET = -0.01342 + 0.75;
		 public static final double RIGHT_BACK_STEER_OFFSET = -0.2842 + 0.25;
		 public static final double LEFT_FRONT_STEER_OFFSET = -0.1741 + 0.25;
		 public static final double RIGHT_FRONT_STEER_OFFSET = 0.2525 - 0.25;
 
		 public static final double SWERVE_CONVERSION_FACTOR = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
 
		 public static final double PHOTON_STDDEV_SCALING_FACTOR = (1.0/2.0);

		 public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
	 }

	 public static final class DDrive {
		 public static final double STARTING_X=0;
		 public static final double STARTING_Y=0;
	}
 
	public static final class Shooter {
		public static final int LEFT_ANGLE_MOTOR_ID = 14; // NEED TO CHANGE
		public static final int RIGHT_ANGLE_MOTOR_ID = 15; // NEED TO CHANGE
		public static final int LEFT_FLYWHEEL_MOTOR_ID = 16; // NEED TO CHANGE
		public static final int RIGHT_FLYWHEEL_MOTOR_ID = 17; // NEED TO CHANGE
		public static final int LEFT_INDEXER_MOTOR_ID = 18; // NEED TO CHANGE
		public static final int RIGHT_INDEXER_MOTOR_ID = 19; // NEED TO CHANGE

		public static final double ANGLE_FEED_FORWARD_KS = 0.0; // NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KG = 0.0; // NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KV = 0.0; // NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_KA = 0.0; // NEED TO CHANGE
		public static final double ANGLE_FEED_FORWARD_VEL = 2.0; // NEED TO TWEAK

		public static final double FLYWHEEL_FEED_FORWARD_KS = 0.0; // NEED TO CHANGE
		public static final double FLYWHEEL_FEED_FORWARD_KV = 0.0; // NEED TO CHANGE
		public static final double FLYWHEEL_FEED_FORWARD_KA = 0.0; // NEED TO CHANGE

    }

	public static final class PhotonvisionConstants {
		public static final double FRONT_CAMERA_HEIGHT_METERS = 0.171; // NOT FINAL
		public static final double BACK_CAMERA_HEIGHT_METERS = 0.146;
		public static final double RIGHT_CAMERA_HEIGHT_METERS = 0.146; 
		public static final double LEFT_CAMERA_HEIGHT_METERS = 0.146;
		public static final double GRID_TARGET_HEIGHT_METERS = 0.36;
		public static final double DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS = 0.59;
		public static final double CAMERA_PITCH_RADIANS = 0; // NOT FINAL
		
		public static final Matrix<N3,N3> CAMERA_MATRIX = MatBuilder.fill(Nat.N3(), Nat.N3(),
			0, 0, 0,
			0, 0, 0,
			0, 0, 0);
		// CAMERA_MATRIX.fill(2664.6321884337376,0.0,-1296.7911812830794,0.0,2585.346203120696,-969.9987887232326,0.0,0.0,1.0);
		// ,"distCoeffs":{"rows":1,"cols":8,"type":6,"data":[-1.2009612471335536,0.21937730784388165,-0.049386955648898626,-0.06465821604835886,0.09917979101966837,-1.2635369876658264,0.30684145946298413,0.06812002422580545]},"calobjectWarp":[4.6580159793967033E-4,9.78523406280745E-4],"observations":[{"locationInObjectSpace":[{"x":0.0,"y":0.0,"z":0.0},{"x":0.02539999969303608,"y":0.0,"z":0.0},{"x":0.05079999938607216,"y":0.0,"z":0.0},{"x":0.07620000094175339,"y":0.0,"z":0.0},{"x":0.10159999877214432,"y":0.0,"z":0.0},{"x":0.12700000405311584,"y":0.0,"z":0.0},{"x":0.15240000188350677,"y":0.0,"z":0.0},{"x":0.0,"y":0.02539999969303608,"z":0.0},{"x":0.02539999969303608,"y":0.02539999969303608,"z":0.0},{"x":0.05079999938607216,"y":0.02539999969303608,"z":0.0},{"x":0.07620000094175339,"y":0.02539999969303608,"z":0.0},{"x":0.10159999877214432,"y":0.02539999969303608,"z":0.0},{"x":0.12700000405311584,"y":0.02539999969303608,"z":0.0},{"x":0.15240000188350677,"y":0.02539999969303608,"z":0.0},{"x":0.0,"y":0.05079999938607216,"z":0.0},{"x":0.02539999969303608,"y":0.05079999938607216,"z":0.0},{"x":0.05079999938607216,"y":0.05079999938607216,"z":0.0},{"x":0.07620000094175339,"y":0.05079999938607216,"z":0.0},{"x":0.10159999877214432,"y":0.05079999938607216,"z":0.0},{"x":0.12700000405311584,"y":0.05079999938607216,"z":0.0},{"x":0.15240000188350677,"y":0.05079999938607216,"z":0.0},{"x":0.0,"y":0.07620000094175339,"z":0.0},{"x":0.02539999969303608,"y":0.07620000094175339,"z":0.0},{"x":0.05079999938607216,"y":0.07620000094175339,"z":0.0},{"x":0.07620000094175339,"y":0.07620000094175339,"z":0.0},{"x":0.10159999877214432,"y":0.07620000094175339,"z":0.0},{"x":0.12700000405311584,"y":0.07620000094175339,"z":0.0},{"x":0.15240000188350677,"y":0.07620000094175339,"z":0.0},{"x":0.0,"y":0.10159999877214432,"z":0.0},{"x":0.02539999969303608,"y":0.10159999877214432,"z":0.0},{"x":0.05079999938607216,"y":0.10159999877214432,"z":0.0},{"x":0.07620000094175339,"y":0.10159999877214432,"z":0.0},{"x":0.10159999877214432,"y":0.10159999877214432,"z":0.0},{"x":0.12700000405311584,"y":0.10159999877214432,"z":0.0},{"x":0.15240000188350677,"y":0.10159999877214432,"z":0.0},{"x":0.0,"y":0.12700000405311584,"z":0.0},{"x":0.02539999969303608,"y":0.12700000405311584,"z":0.0},{"x":0.05079999938607216,"y":0.12700000405311584,"z":0.0},{"x":0.07620000094175339,"y":0.12700000405311584,"z":0.0},{"x":0.10159999877214432,"y":0.12700000405311584,"z":0.0},{"x":0.12700000405311584,"y":0.12700000405311584,"z":0.0},{"x":0.15240000188350677,"y":0.12700000405311584,"z":0.0},{"x":0.0,"y":0.15240000188350677,"z":0.0},{"x":0.02539999969303608,"y":0.15240000188350677,"z":0.0},{"x":0.05079999938607216,"y":0.15240000188350677,"z":0.0},{"x":0.07620000094175339,"y":0.15240000188350677,"z":0.0},{"x":0.10159999877214432,"y":0.15240000188350677,"z":0.0},{"x":0.12700000405311584,"y":0.15240000188350677,"z":0.0},{"x":0.15240000188350677,"y":0.15240000188350677,"z":0.0}]
		public static final Matrix<N5,N1> DIST_COEFFS = null; // TODO: Calibrate

		public static final Transform3d ROBOT_TO_FRONT_CAMERA= new Transform3d(new Translation3d(0.381, 0, FRONT_CAMERA_HEIGHT_METERS),new Rotation3d(0.0,0.0,0.0));
		public static final Transform3d ROBOT_TO_BACK_CAMERA= new Transform3d(new Translation3d(-0.381,0,BACK_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(180),0));
		public static final Transform3d ROBOT_TO_LEFT_CAMERA= new Transform3d(new Translation3d(0,0.381,LEFT_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(90),0));
		public static final Transform3d ROBOT_TO_RIGHT_CAMERA= new Transform3d(new Translation3d(0,-0.381,RIGHT_CAMERA_HEIGHT_METERS),new Rotation3d(0,Units.degreesToRadians(270),0));

		public static final int BEAM_BREAK_ID = 0;
	}

	public static final class Intake {

		public static final int INDEX_MOTOR_ID = 0;
		public static final int LEFT_CENTER_ID = 1;
		public static final int RIGHT_CENTER_ID = 2;

		public static final int BEAM_PORT = 4;
		public static final int LEFT_PIVOT_MOTOR_ID = 2; // NEED TO CHANGE
		public static final int RIGHT_PIVOT_MOTOR_ID = 5; // NEED TO CHANGE
		public static final int BELT_MOTOR_ID = 17; // NEED TO CHANGE
		public static final int PIVOT_ENCODER_ID = 3; // NEED TO CHANGE
		public static final int HOME_LIMIT_SWITCH_ID = 0; // NEED TO CHANGE
		public static final int GROUND_LIMIT_SWITCH_ID = 1; // NEED TO CHANGE

		public static final double PIVOT_FEED_FORWARD_KS = 0.0; // NEED TO CHANGE
		public static final double PIVOT_FEED_FORWARD_KG = 0.932; // NEED TO CHANGE
		public static final double PIVOT_FEED_FORWARD_KV = 0.335; // NEED TO CHANGE
		public static final double PIVOT_FEED_FORWARD_KA = 0.1; // NEED TO CHANGE

		public static final double PIVOT_PID_CONTROLLER_P = 1.5; // NEED TO CHANGE
		public static final double PIVOT_PID_CONTROLLER_I = 0.0; // NEED TO CHANGE
		public static final double PIVOT_PID_CONTROLLER_D = 0.0; // NEED TO CHANGE

		public static final double SIMULATION_OFFSET = 0.0; // NEED TO CHANGE
		public static final double SIMULATION_SCALE = 0.0; // NEED TO CHANGE
		public static final double INTAKE_SIM_LENGTH = 0.0; // NEED TO CHANGE

		public static final double HOME_ANGLE = Math.PI/2;
		public static final double GROUND_ANGLE = Math.toRadians(345);// GET ANGLE FROM JOHN

		public static final double GEAR_RATIO = (1/9); //GET FROM SOMEONE

		public static final double ROTATION_SCALE = 3;
	}
}
