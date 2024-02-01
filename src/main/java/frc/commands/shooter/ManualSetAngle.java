package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import static frc.subsystems.Shooter.INTERPOLATED_ANGLE;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualSetAngle extends Command {    
    double desiredAngle;
    double desiredVelocityRight;
    double desiredVelocityLeft;

    public ManualSetAngle(double height, double range) {
        addRequirements(shooter);
        
        // desiredVelocityRight = 17;
        // desiredVelocityLeft = 17 * shooter.getFactor();
        //desiredAngle = Math.atan(2 *height / range);
        //Calculate Speed here;
        //desiredVelocity = 0.0;
        //desiredVelocity = Math.sqrt((2*height *9.8*(16*height * height + range * range ))/(8*height));
        //desiredVelocityRight = Math.sqrt(2 * height * 9.8) / Math.sin(desiredAngle); //new equation
        // desiredVelocityLeft = 0;

        desiredVelocityRight = 32;
        desiredAngle = shooter.interpolateAngle(range);
        INTERPOLATED_ANGLE.setDouble(desiredAngle);
    }


    @Override
    public void execute() {
        // shooter.leftAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle));
        // shooter.rightAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle));
        desiredVelocityLeft = desiredVelocityRight * shooter.getFactor();

        //shooter.leftFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeedLeft(desiredVelocityLeft));
        //shooter.rightFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeedRight(desiredVelocityRight));
    } 

    // @Override
    // public boolean isFinished() {
    //     return shooter.getShooterAngle() == desiredAngle && shooter.getVelocitySpeed() == desiredVelocity;
    // }

}