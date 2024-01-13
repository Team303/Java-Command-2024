package frc.subsystems;

import com.ctre.phoenixpro.hardware.CANcoder;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public class ShooterSubsystem {
    public CANcoder flywheelMotor = new CANcoder(RobotMap.Turret.FLYWHEEL_MOTOR_ID); // For speeding up/shooting  the game piece
    public CANcoder shooterToAngleMotor = new CANcoder(4);
    public static final ShuffleboardTab TURRET_TAB = Shuffleboard.getTab("Turret"); //Shuffleboard tab
    public static final GenericEntry flywheelVelocityEntry = TURRET_TAB.add("Flywheel Velocity", 0).withPosition(0, 0).getEntry(); //post to shuffleboard
    public double xVisionDistance = 20; //arbitrary photonvision value (fixed)

    


}