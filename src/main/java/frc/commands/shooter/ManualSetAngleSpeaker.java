package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import static frc.subsystems.Shooter.INTERPOLATED_DEGREES_ENTRY;
import frc.robot.RobotMap;
import frc.robot.RobotMap.FieldConstants;
import frc.robot.util.FieldRelativeAcceleration;
import frc.robot.util.FieldRelativeSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualSetAngleSpeaker extends Command {    
    double desiredAngle;
    double shotTime;
    double desiredVelocityRight;
    double desiredVelocityLeft;
    // will work once merged with master
    Translation2d target = FieldConstants.centerSpeakOpening.getTranslation();
    // double range = FieldConstants.centerSpeakOpening.getTranslation().getDistance(swerve.getPose());
    double range = 0;

    //TODO: Make the range/height not a parameter once merged into master
    public ManualSetAngleSpeaker(double height, double range) {
        addRequirements(shooter);
        //shooter.resetEncoders();
        
        // desiredVelocityRight = 17;
        // desiredVelocityLeft = 17 * shooter.getFactor();
        //desiredAngle = Math.atan(2 *height / range);
        //Calculate Speed here;
        //desiredVelocity = 0.0;
        //desiredVelocity = Math.sqrt((2*height *9.8*(16*height * height + range * range ))/(8*height));
        //desiredVelocityRight = Math.sqrt(2 * height * 9.8) / Math.sin(desiredAngle); //new equation
        desiredVelocityRight = 31;

        desiredAngle = shooter.interpolateAngle(range);
        INTERPOLATED_DEGREES_ENTRY.setDouble(Math.toDegrees((desiredAngle)));

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

        shooter.leftFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeedLeft(desiredVelocityLeft));
        //shooter.rightFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeedRight(desiredVelocityRight));

        // shooter.leftAngleMotor.setVoltage(shooter.calculateAngleSpeedLeft(desiredAngle));
        shooter.rightAngleMotor.setVoltage(shooter.calculateAngleSpeedRight(desiredAngle));
    } 

    @Override
    public boolean isFinished() {
        return shooter.atHardLimit();
    }

    public void end(boolean interrupted) {
        shooter.leftFlywheelMotor.setVoltage(0);
    }


}