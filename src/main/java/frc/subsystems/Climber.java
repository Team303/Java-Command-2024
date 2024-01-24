package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final TalonFX armOne;
    private final TalonFX armTwo;

    private final double armOneUpperLimit;
    private final double armTwoUpperLimit;

    public Climber(double armOneLimit, double armTwoLimit) {
        armOne = new TalonFX(0);
        armTwo = new TalonFX(1);

        armOneUpperLimit = armOneLimit;
        armTwoUpperLimit = armTwoLimit;

        resetEncoders();
    }

    public void resetEncoders() {
        armOne.setSelectedSensorPosition(0);
        armTwo.setSelectedSensorPosition(0);
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
