package frc.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
    private final TalonFX armOne;
    private final TalonFX armTwo;

    private final double armOneUpperLimit;
    private final double armTwoUpperLimit;

    private final DigitalInput armOneLimitSwitch;
    private final DigitalInput armTwoLimitSwitch;

    public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");

    public static final GenericEntry ARM_ONE_ENC = CLIMBER_TAB.add("Arm One Position", 0)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();

    public static final GenericEntry ARM_TWO_ENC = CLIMBER_TAB.add("Arm Two Position", 0)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();

    public static final GenericEntry ARM_ONE_SWITCH = CLIMBER_TAB.add("Arm One Limit Switch", false)
            .withPosition(3, 0)
            .withSize(2, 1)
            .getEntry();

    public static final GenericEntry ARM_TWO_SWITCH = CLIMBER_TAB.add("Arm One Limit Switch", false)
            .withPosition(3, 1)
            .withSize(2, 1)
            .getEntry();

    public Climber() {
        armOne = new TalonFX(0);
        armTwo = new TalonFX(1);

        FeedbackConfigs FC = new FeedbackConfigs();
        FC.RotorToSensorRatio = RobotMap.Climber.ROTOR_TO_SENSOR_FACTOR; 
        FC.SensorToMechanismRatio = RobotMap.Climber.SENSOR_TO_MECHANISM_FACTOR; // not sure which one the conversion factor is for lol

        armOne.getConfigurator().apply(FC, 0.030);
        armTwo.getConfigurator().apply(FC, 0.030);

        CurrentLimitsConfigs CLC = new CurrentLimitsConfigs();
        CLC.StatorCurrentLimit = 40; 
        CLC.SupplyCurrentLimit = 40; 

        armOne.getConfigurator().apply(CLC);
        armTwo.getConfigurator().apply(CLC);

        // armOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30); i think by default it should be integrated sensor so im not gonna do this
                                                                                     
        // armOne.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
        // armOne.configVelocityMeasurementWindow(60);
        // armTwo.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
        // armTwo.configVelocityMeasurementWindow(60);

        /*
         * In Phoenix 6, the velocity rolling average window in Talon FX and CANcoder has been replaced with a Kalman filter, 
         * resulting in a less noisy velocity signal with a minimal impact on latency. 
         * As a result, the velocity measurement period/window configs are no longer necessary in Phoenix 6 and have been removed.
         */

        armOneUpperLimit = RobotMap.Climber.ARM_ONE_LIMIT;
        armTwoUpperLimit = RobotMap.Climber.ARM_TWO_LIMIT;

        // Limit switches
        armOneLimitSwitch = new DigitalInput(RobotMap.Climber.ARM_ONE_LIMIT_SWITCH);
        armTwoLimitSwitch = new DigitalInput(RobotMap.Climber.ARM_TWO_LIMIT_SWITCH);

        resetEncoders();
    }

    public void resetEncoders() {
        armOne.setPosition(0);
        armTwo.setPosition(0); // think it's supposed to be armTwo
    }

    public void extendArms(double speed) {
        double armOneSpeed = getArmEncoderPosition(1) < armOneUpperLimit ? speed : 0; // is the speed in volts? 
        double armTwoSpeed = getArmEncoderPosition(2) < armTwoUpperLimit ? speed : 0;
        armOne.setControl(new VoltageOut(armOneSpeed).withEnableFOC(true).withOverrideBrakeDurNeutral(true)); // migration docs said that this is replacement, but the prev example makes no sense, read comment below 
        armTwo.setControl(new VoltageOut(armTwoSpeed).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
        // armOne.set(ControlMode.PercentOutput, armOneSpeed); // isn't percentoutput based off of voltagecompensation? we haven't even set it here tho
        // armTwo.set(ControlMode.PercentOutput, armTwoSpeed);
    }

    public void keepGoing(double speed, double limit) {
        double armOneSpeed = getArmEncoderPosition(1) > limit? speed: 0;
        double armTwoSpeed = getArmEncoderPosition(2) > limit? speed: 0;
        armOne.setControl(new VoltageOut(armOneSpeed).withEnableFOC(true).withOverrideBrakeDurNeutral(true)); 
        armTwo.setControl(new VoltageOut(armTwoSpeed).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public void retractArms(double speed) {
        speed = -Math.abs(speed);
        double armOneSpeed = !isArmAtLowerLimit(1) ? speed : 0; 
        double armTwoSpeed = !isArmAtLowerLimit(2) ? speed : 0;
        armOne.setControl(new VoltageOut(armOneSpeed).withEnableFOC(true).withOverrideBrakeDurNeutral(true)); 
        armTwo.setControl(new VoltageOut(armTwoSpeed).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public double getArmEncoderPosition(int armIndex) {
        return armIndex == 1 ? armOne.getPosition().refresh().getValue() : armTwo.getPosition().refresh().getValue();
    }

    public boolean isArmAtLimit(int armIndex) {
        return armIndex == 1 ? getArmEncoderPosition(1) >= armOneUpperLimit
                : getArmEncoderPosition(2) >= armTwoUpperLimit;
    }

    public boolean isArmAtLowerLimit(int armIndex) {
        return armIndex == 1 ? armOneLimitSwitch.get() : armTwoLimitSwitch.get();
    }

    public void stopArms() {
        armOne.setControl(new VoltageOut(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
        armTwo.setControl(new VoltageOut(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    @Override
    public void periodic() {
        System.out.println("Aritra code bangs");

        ARM_ONE_ENC.setDouble(armOne.getPosition().refresh().getValue());
        ARM_TWO_ENC.setDouble(armTwo.getPosition().refresh().getValue());

        ARM_ONE_SWITCH.setBoolean(armOneLimitSwitch.get());
        ARM_TWO_SWITCH.setBoolean(armTwoLimitSwitch.get());
    }   
}

// public class Climber extends SubsystemBase {

// private final TalonFX armOne;
// private final TalonFX armTwo;

// private final DigitalInput armOneLimitSwitch;
// private final DigitalInput armTwoLimitSwitch;

// public Climber() {
// armOne = new TalonFX(0);
// armTwo = new TalonFX(1);

// armOneLimitSwitch = new DigitalInput(0);
// armTwoLimitSwitch = new DigitalInput(1);
// }

// public void setArmOneSpeed(double speed) {
// if (!isArmOneAtLimit()) {
// armTwo.set(speed);
// } else {
// armTwo.set(0);
// }
// }

// public void setArmTwoSpeed(double speed) {
// if (!isArmTwoAtLimit()) {
// armTwo.set(speed);
// } else {
// armTwo.set(0);
// }
// }

// public void stopArms() {
// armOne.set(0);
// armTwo.set(0);
// }

// public void extendArms() {
// double speed = 1.0;
// setArmOneSpeed(speed);
// setArmTwoSpeed(speed);
// }

// public boolean isArmOneAtLimit() {
// return armOneLimitSwitch.get();
// }

// public boolean isArmTwoAtLimit() {
// return armTwoLimitSwitch.get();
// }

// @Override
// public void periodic() {
// // Called once per scheduler run
// }
// }