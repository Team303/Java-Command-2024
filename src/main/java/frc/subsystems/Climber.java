package frc.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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

        armOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30); // PIDx is normally 0, Timeout is normally 30ms
        armOne.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
        armOne.configVelocityMeasurementWindow(60);
        armOne.configSelectedFeedbackCoefficient(conversionFactor, 0, 30);

        armTwo.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
        armTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30); // PIDx is normally 0, Timeout is normally 30ms
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
        if (getArmOneEncoderPosition() < armOneUpperLimit) {
            armOne.set(ControlMode.PercentOutput, speed);
        } else {
            armOne.set(ControlMode.PercentOutput, 0);
        }

        if (getArmTwoEncoderPosition() < armTwoUpperLimit) {
            armTwo.set(ControlMode.PercentOutput, speed);
        } else {
            armTwo.set(ControlMode.PercentOutput, 0);
        }
    }

    public void retractArms(double speed) {
        speed = -Math.abs(speed);

        if (!isArmOneAtLowerLimit()) {
            armOne.set(ControlMode.PercentOutput, speed);
        } else {
            armOne.set(ControlMode.PercentOutput, 0);
        }
        if (!isArmTwoAtLowerLimit()) {
            armTwo.set(ControlMode.PercentOutput, speed);
        } else {
            armTwo.set(ControlMode.PercentOutput, 0);
        }
    }

    public double getArmOneEncoderPosition() {
        return armOne.getSelectedSensorPosition();
    }

    public double getArmTwoEncoderPosition() {
        return armTwo.getSelectedSensorPosition();
    }

    public boolean isArmOneAtLimit() {
        return getArmOneEncoderPosition() >= armOneUpperLimit;
    }

    public boolean isArmTwoAtLimit() {
        return getArmTwoEncoderPosition() >= armTwoUpperLimit;
    }

    public boolean isArmOneAtLowerLimit() {
        return armOneLimitSwitch.get();
    }

    public boolean isArmTwoAtLowerLimit() {
        return armTwoLimitSwitch.get();
    }

    public void stopArms() {
        armOne.set(ControlMode.PercentOutput, 0);
        armTwo.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        // Monitor and adjust motor based on sensor readings, if you want lol
    }
}

// public class Climber extends SubsystemBase {
    
//     private final TalonFX armOne;
//     private final TalonFX armTwo;

//     private final DigitalInput armOneLimitSwitch;
//     private final DigitalInput armTwoLimitSwitch;

//     public Climber() {
//         armOne = new TalonFX(0);
//         armTwo = new TalonFX(1);

//         armOneLimitSwitch = new DigitalInput(0);
//         armTwoLimitSwitch = new DigitalInput(1);
//     }

//     public void setArmOneSpeed(double speed) {
//         if (!isArmOneAtLimit()) {
//             armTwo.set(speed);
//         } else {
//             armTwo.set(0);
//         }
//     }

//     public void setArmTwoSpeed(double speed) {
//         if (!isArmTwoAtLimit()) {
//             armTwo.set(speed);
//         } else {
//             armTwo.set(0);
//         }
//     }

//     public void stopArms() {
//         armOne.set(0);
//         armTwo.set(0);
//     }

//     public void extendArms() {
//         double speed = 1.0;
//         setArmOneSpeed(speed);
//         setArmTwoSpeed(speed);
//     }

//     public boolean isArmOneAtLimit() {
//         return armOneLimitSwitch.get();
//     }

//     public boolean isArmTwoAtLimit() {
//         return armTwoLimitSwitch.get();
//     }

//     @Override
//     public void periodic() {
//         // Called once per scheduler run
//     }
// }
