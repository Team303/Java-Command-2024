package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import static frc.subsystems.Shooter.INTERPOLATED_DEGREES_ENTRY;
import static frc.subsystems.Shooter.DESIRED_LEFT_RPM_ENTRY;
import static frc.subsystems.Shooter.DESIRED_RIGHT_RPM_ENTRY;

import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.RobotMap;
import frc.robot.RobotMap.FieldConstants;
import frc.robot.util.FieldRelativeAcceleration;
import frc.robot.util.FieldRelativeSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetShooterSpeaker extends Command {    
    double desiredAngle;
    double shotTime;
    double desiredVelocityRight;
    double desiredVelocityLeft;
    // will work once merged with master
    Translation2d target = FieldConstants.centerSpeakOpening.getTranslation();
    // double range = FieldConstants.centerSpeakOpening.getTranslation().getDistance(swerve.getPose());
    double range = 30;

    //TODO: Make the range/height not a parameter once merged into master
    public SetShooterSpeaker(double height, double range) {
        addRequirements(shooter);
        
        desiredVelocityRight = 22;

        desiredAngle = shooter.interpolateAngle(range);
        INTERPOLATED_DEGREES_ENTRY.setDouble(Math.toDegrees((desiredAngle)));
        shooter.pivotAngle = Math.toDegrees(desiredAngle);

        //FieldRelativeSpeeds curVel = swerve.currentSpeed;
        //FieldRelativeAcceleration curAccel = swerve.currentAccel;
        double desiredTime = shooter.interpolateTime(range);
        //TODO: Uncomment once merged into master
        // Translation2d movingGoalLocation = new Translation2d();
        
        // for(int i=0;i<5;i++){
        //     double virtualGoalX = target.getX()
        //             - desiredTime * (curVel.vx + curAccel.ax * RobotMap.Shooter.accelerationCompensationFactor);
        //     double virtualGoalY = target.getY()
        //             - desiredTime * (curVel.vy + curAccel.ay * RobotMap.Shooter.accelerationCompensationFactor);

        //     SmartDashboard.putNumber("Goal X", virtualGoalX);
        //     SmartDashboard.putNumber("Goal Y", virtualGoalY);

        //     Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

        //     Translation2d toTestGoal = testGoalLocation.minus(swerve.getPose().getTranslation());

        //     double newShotTime = shooter.interpolateTime(toTestGoal.getDistance(new Translation2d()));

        //     if(Math.abs(newShotTime-desiredTime) <= 0.010){
        //         i=4;
        //     }
            
        //     if(i == 4){
        //         movingGoalLocation = testGoalLocation;
        //         SmartDashboard.putNumber("NewShotTime", newShotTime);
        //     }
        //     else{
        //         desiredTime = newShotTime;
        //     }

        // }
        // double newDist = movingGoalLocation.minus(swerve.getPose().getTranslation()).getDistance(new Translation2d());

        // SmartDashboard.putNumber("NewDist", newDist);
    }


    @Override
    public void execute() {
        desiredVelocityLeft = desiredVelocityRight * shooter.getFactor();

        // double leftCalcVelocity = shooter.calculateFlywheelSpeedLeft(desiredVelocityLeft /  (2 * Math.PI * 0.0508));
        // double rightCalcVelocity = shooter.calculateFlywheelSpeedRight(desiredVelocityRight /  (2 * Math.PI * 0.0508));
        DESIRED_LEFT_RPM_ENTRY.setDouble(desiredVelocityLeft / (2 * Math.PI * 0.0508) * 60);        
        DESIRED_RIGHT_RPM_ENTRY.setDouble(desiredVelocityRight / (2 * Math.PI * 0.0508) * 60);

        shooter.leftFlywheelMotor.setControl(shooter.flywheelVoltageLeft.withVelocity(-(desiredVelocityLeft / (2 * Math.PI * 0.0508))));
        shooter.rightFlywheelMotor.setControl(shooter.flywheelVoltageRight.withVelocity(desiredVelocityRight /  (2 * Math.PI * 0.0508)));

        shooter.leftAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle)); 
        //shooter.rightAngleMotor.setVoltage(shooter.calculateAngleSpeedRight(desiredAngle)); --> Dont need because follower
    } 

    @Override
    public boolean isFinished() {
        return shooter.atHardLimit();
    }

    public void end(boolean interrupted) {
        //shooter.leftFlywheelMotor.setVoltage(0);
    }


}