package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
    private final TalonFX armOne;
    private final TalonFX armTwo;

    private final double armOneUpperLimit;
    private final double armTwoUpperLimit;

    private final DigitalInput armOneLimitSwitch;
    private final DigitalInput armTwoLimitSwitch;

    public Climber() {
        double conversionFactor = 1 / 2048.0; // arbitrary value - change to the actual CPR
        armOne = new TalonFX(0);
        armTwo = new TalonFX(1);

        armOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30); // PIDx is normally 0, Timeout is
                                                                                     // normally 30ms
        armOne.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
        armOne.configVelocityMeasurementWindow(60);
        armOne.configSelectedFeedbackCoefficient(conversionFactor, 0, 30);

        armTwo.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
        armTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30); // PIDx is normally 0, Timeout is
                                                                                     // normally 30ms
        armTwo.configVelocityMeasurementWindow(60);
        armTwo.configSelectedFeedbackCoefficient(conversionFactor, 0, 30);

        armOneUpperLimit = RobotMap.Swerve.ARM_ONE_LIMIT;
        armTwoUpperLimit = RobotMap.Swerve.ARM_TWO_LIMIT;

        // Limit switches
        armOneLimitSwitch = new DigitalInput(RobotMap.Swerve.ARM_ONE_LIMIT_SWITCH);
        armTwoLimitSwitch = new DigitalInput(RobotMap.Swerve.ARM_TWO_LIMIT_SWITCH);

        resetEncoders();
    }

    public void resetEncoders() {
        armOne.setSelectedSensorPosition(0);
        armOne.setSelectedSensorPosition(0);
    }

    public void extendArms(double speed) {
        double armOneSpeed = getArmEncoderPosition(1) < armOneUpperLimit ? speed : 0;
        double armTwoSpeed = getArmEncoderPosition(2) < armTwoUpperLimit ? speed : 0;
        armOne.set(ControlMode.PercentOutput, armOneSpeed);
        armTwo.set(ControlMode.PercentOutput, armTwoSpeed);
    }

    public void retractArms(double speed) {
        speed = -Math.abs(speed);
        double armOneSpeed = !isArmAtLowerLimit(1) ? speed : 0;
        double armTwoSpeed = !isArmAtLowerLimit(2) ? speed : 0;
        armOne.set(ControlMode.PercentOutput, armOneSpeed);
        armTwo.set(ControlMode.PercentOutput, armTwoSpeed);
    }

    public double getArmEncoderPosition(int armIndex) {
        return armIndex == 1 ? armOne.getSelectedSensorPosition() : armTwo.getSelectedSensorPosition();
    }

    public boolean isArmAtLimit(int armIndex) {
        return armIndex == 1 ? getArmEncoderPosition(1) >= armOneUpperLimit
                : getArmEncoderPosition(2) >= armTwoUpperLimit;
    }

    public boolean isArmAtLowerLimit(int armIndex) {
        return armIndex == 1 ? armOneLimitSwitch.get() : armTwoLimitSwitch.get();
    }

    public void stopArms() {
        armOne.set(ControlMode.PercentOutput, 0);
        armTwo.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        System.out.println("Aritra code bangs");
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
