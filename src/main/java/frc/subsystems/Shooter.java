package frc.subsystems;

import static frc.robot.Robot.shooter;

import java.util.HashMap;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.RobotMap;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {

    //Flywheel stuff
    public final TalonFX leftFlywheelMotor;
    public final TalonFX rightFlywheelMotor;

    public final VelocityVoltage flywheelVoltageLeft = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    public final VelocityVoltage flywheelVoltageRight = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    //Angle stuff
    public final TalonFX leftAngleMotor;
    public final TalonFX rightAngleMotor;

    public final ProfiledPIDController anglePIDController;
    public final ArmFeedforward angleFeedForward;

    public final DutyCycleEncoder angleEncoder;

    //Limit Switch Stuff
    //public final DigitalInput angleLimitSwitch;

    //Interploation Stuff
    public InterpolatingDoubleTreeMap angleInterpolator21 = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap angleInterpolator17 = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap timeInterpolator21 = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap timeInterpolator17 = new InterpolatingDoubleTreeMap();


    //Interpolation Table 1: 21.27 meters per second 💀
    public static double[] interpolationAngles21 = {1.060, 0.961, 0.876, 0.802, 0.738, 0.683, 0.635, 0.594, 0.559, 0.527, 0.500,  //1.00 --> 3.750
                                                    0.476, 0.454, 0.435, 0.419, 0.404, 0.390, 0.378, 0.367, 0.357, 0.349, 0.341,  //3.00 --> 5.750     
                                                    0.333, 0.327, 0.321, 0.316, 0.311, 0.307, 0.303, 0.299, 0.296, 0.293, 0.291,  //6.00 --> 8.750
                                                    0.289, 0.287, 0.285, 0.284, 0.282, 0.281, 0.280, 0.280, 0.279, 0.279, 0.278,  //9.00 --> 11.75
                                                    0.278};                                                                       //12.0 --> 12.00
    //Interpolation Table 2: 17 meters per second 
    public static double[] interpolationAngles17 = {1.0642, 0.9670, 0.8827, 0.8103, 0.7481, 0.6949, 0.6491, 0.6097, 0.5756, 0.5461, 0.5204,  //1.00 --> 3.750
                                                    0.4980, 0.4784, 0.4612, 0.4460, 0.4327, 0.4210, 0.4107, 0.4015, 0.3935, 0.3864, 0.3802,  //3.00 --> 5.750
                                                    0.3748, 0.3701, 0.3660, 0.3625, 0.3595, 0.3569, 0.3548, 0.3531, 0.3518, 0.3508, 0.3501,  //6.00 --> 8.750
                                                    0.3497, 0.3496, 0.3497, 0.3501, 0.3507, 0.3515, 0.3525, 0.3537, 0.3550, 0.3565, 0.3582,  //9.00 --> 11.75
                                                    0.3601};                                                                                 //12.0 --> 12.00


    public static double[] interpolationTimes21 = new double[interpolationAngles21.length];
    public static double[] interpolationTimes17 = new double[interpolationAngles17.length];

    //Shuffleboard Stuff
    public static final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("Shooter"); //Shuffleboard tab

        //Flywheel Entries
        public static final GenericEntry DESIRED_LEFT_RPM_ENTRY = SHOOTER_TAB.add("Desired Left RPM", 0.0).withPosition(0, 0).withSize(2, 1).getEntry();
        public static final GenericEntry DESIRED_RIGHT_RPM_ENTRY = SHOOTER_TAB.add("Desired Right RPM", 0.0).withPosition(0, 2).withSize(2, 1).getEntry();
        public static final GenericEntry ACTUAL_LEFT_RPM_ENTRY = SHOOTER_TAB.add("Actual Left RPM", 0.0).withPosition(1, 0).withSize(2, 1).getEntry();
        public static final GenericEntry ACTUAL_RIGHT_RPM_ENTRY = SHOOTER_TAB.add("Actual Right RPM", 0.0).withPosition(1, 2).withSize(2, 1).getEntry();
        public static final GenericEntry RPM_DIFF_FACTOR_ENTRY = SHOOTER_TAB.add("RPM Diff Factor", 0.0).withPosition(2, 0).withSize(1, 1).getEntry();

        //Angle Entries
        public static final GenericEntry INTERPOLATED_DEGREES_ENTRY = SHOOTER_TAB.add("Desired Degrees", 0.0).withPosition(2, 1).withSize(1, 1).getEntry();
        public static final GenericEntry ACTUAL_SHOOTER_ANGLE_ENTRY = SHOOTER_TAB.add("Actual Degrees", 0.0).withPosition(5, 0).getEntry();

        //Limit Entries
        public static final GenericEntry ANGLE_LIMIT_SWITCH_STATUS_ENTRY = SHOOTER_TAB.add("Angle Hard Limit", false).withPosition(0, 0).getEntry();

    //Extras
    public double diffFactor = 1.0;
    public double pivotAngle = 0.0;

    public Shooter() {
        //Flywheel Initalization

        leftFlywheelMotor = new TalonFX(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID);
        rightFlywheelMotor = new TalonFX(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID);

        leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        leftFlywheelMotor.setInverted(false);
        rightFlywheelMotor.setInverted(true);

        TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();

        flywheelConfigs.Slot0.kP = 0.3;
        flywheelConfigs.Slot0.kI = 40;
        flywheelConfigs.Slot0.kD = 0.0000;
        flywheelConfigs.Slot0.kV = 0;


        flywheelConfigs.Voltage.PeakForwardVoltage = 12;
        flywheelConfigs.Voltage.PeakReverseVoltage = -12;

        leftFlywheelMotor.getConfigurator().apply(flywheelConfigs);
        rightFlywheelMotor.getConfigurator().apply(flywheelConfigs);

        MotorOutputConfigs alanbbc = new MotorOutputConfigs();
        alanbbc.withInverted(InvertedValue.Clockwise_Positive);
        rightFlywheelMotor.getConfigurator().apply(alanbbc);

        MotorOutputConfigs aritrabbc = new MotorOutputConfigs();
        aritrabbc.withInverted(InvertedValue.Clockwise_Positive);
        

        //Angle Initalization
        leftAngleMotor = new TalonFX(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID);
        rightAngleMotor = new TalonFX(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID);

        leftAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        rightAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        rightAngleMotor.setControl(new Follower(leftAngleMotor.getDeviceID(), true));

        angleFeedForward = new ArmFeedforward(RobotMap.Shooter.ANGLE_FEED_FORWARD_KS, 
                                              RobotMap.Shooter.ANGLE_FEED_FORWARD_KG, 
                                              RobotMap.Shooter.ANGLE_FEED_FORWARD_KV, 
                                              RobotMap.Shooter.ANGLE_FEED_FORWARD_KA);


        // go up to 90 degrees in one second 1/4 second to accelerate to max speed
        TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(Math.PI/2, 
            Math.PI * 4);

        anglePIDController = new ProfiledPIDController(RobotMap.Shooter.ANGLE_PID_CONTROLLER_P, 
                                                       RobotMap.Shooter.ANGLE_PID_CONTROLLER_I,
                                                       RobotMap.Shooter.ANGLE_PID_CONTROLLER_D, 
                                                       pidConstraints);

        anglePIDController.enableContinuousInput(0, 2 * Math.PI);
        anglePIDController.setTolerance(Math.toRadians(2.0));
    
        angleEncoder = new DutyCycleEncoder(RobotMap.Shooter.ANGLE_ENCODER_ID);

        //Limit Switch Initalization
        //angleLimitSwitch = new DigitalInput(RobotMap.Shooter.ANGLE_HARD_STOP_SWITCH_ID);

        //Interpolation Initalization
        initalizeInterpolationTable21();
        initalizeInterpolationTable17();

        //Current Limit Initalization
        CurrentLimitsConfigs clc40 = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);
        //CurrentLimitsConfigs clc30 = new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30);
        

        leftFlywheelMotor.getConfigurator().apply(clc40);
        rightFlywheelMotor.getConfigurator().apply(clc40);

        leftAngleMotor.getConfigurator().apply(clc40);
        leftAngleMotor.getConfigurator().apply(aritrabbc);
        rightAngleMotor.getConfigurator().apply(clc40);

        anglePIDController.reset(0);

    }


    //Flywheel Functions
    public double getVelocitySpeedLeft() {

        // return speed in rot/sec

        return leftFlywheelMotor.getVelocity().refresh().getValueAsDouble();
    }

    public double getVelocitySpeedRight() {
        // return speed in rot/sec
        return rightFlywheelMotor.getVelocity().refresh().getValueAsDouble();
    }


    //Angle Functions
    public double calculateAngleSpeed(double angle) {


        final double angleOutput = anglePIDController.calculate(getAbsoluteShooterAngle(), angle);
        final double angleFeedforward = angleFeedForward.calculate(angle, anglePIDController.getSetpoint().velocity);
        return angleOutput + angleFeedforward;
    }

    public boolean atSetpoint(){
        return anglePIDController.atSetpoint();
    }

    private double normalizeAngle(double angleRad) {
		angleRad %= Math.PI * 2;

		if (angleRad < 0)
			angleRad += Math.PI * 2;
		
		return angleRad;
	}

    public double getAbsoluteShooterAngle() {

        //get angular position in rad

        return normalizeAngle(angleEncoder.getAbsolutePosition() * 2 * Math.PI - Math.toRadians(140));

    }

    public double getAngleMotorVelocity() {

        //get angular velocity in rad/sec

        return leftAngleMotor.getVelocity().refresh().getValueAsDouble() * RobotMap.Shooter.ANGLE_CONVERSION_FACTOR * 2 * Math.PI;
    }

	public boolean atHardLimit() {
        return false;
        //return !angleLimitSwitch.get();
    }

    //Interpolation Functions
    public void initalizeInterpolationTable21() {
        for(int i=0;i<interpolationAngles21.length;i++){
            angleInterpolator21.put((double)i*0.25+1.0,interpolationAngles21[i]);
        }
        for(int i=0;i<interpolationTimes21.length;i++){
            timeInterpolator21.put((double)i*0.25+1.0,interpolationTimes21[i]);
        }
    }

    public void initalizeInterpolationTable17() {
        for(int i=0;i<interpolationAngles17.length;i++){
            angleInterpolator17.put((double)i*0.25+1.0,interpolationAngles17[i]);
        }
        for(int i=0;i<interpolationTimes17.length;i++){
            timeInterpolator17.put((double)i*0.25+1.0,interpolationTimes17[i]);
        }
    }

    public double interpolateAngle(double range, boolean is21) {

        return is21 ? angleInterpolator21.get(range) : angleInterpolator17.get(range);
    }

    public double interpolateTime(double range, boolean is21) {

        return is21 ? timeInterpolator21.get(range) : timeInterpolator17.get(range);
    }

    //Factor Functions
    public double getFactor() {
        return diffFactor;
    }

    public void setFactor(double factor) {
        diffFactor = factor;
    }

    @Override
    public void periodic() {
        //Flywheel Entries

        //scale by 60 for rpm
        ACTUAL_LEFT_RPM_ENTRY.setDouble(-getVelocitySpeedLeft() * 60);
        ACTUAL_RIGHT_RPM_ENTRY.setDouble(getVelocitySpeedRight() * 60);
        RPM_DIFF_FACTOR_ENTRY.setDouble(diffFactor);
 
        //Angle Entries
        ACTUAL_SHOOTER_ANGLE_ENTRY.setDouble(getAbsoluteShooterAngle() * (180/Math.PI));

        //Limit Entries
        ANGLE_LIMIT_SWITCH_STATUS_ENTRY.setBoolean(atHardLimit());
    }
    
}