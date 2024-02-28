package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FieldRelativeSpeeds {
    public double vx;
    public double vy;
    public double omega;

    public FieldRelativeSpeeds(double vx, double vy, double omega){
        this.vx=vx;
        this.vy=vy;
        this.omega=omega;
    }

    public FieldRelativeSpeeds(){
        this(0.0,0.0,0.0);
    }

    public FieldRelativeSpeeds(ChassisSpeeds chassis, Rotation2d gyroAngle){
        this(chassis.vxMetersPerSecond * gyroAngle.getCos() - chassis.vyMetersPerSecond * gyroAngle.getSin(),
                chassis.vyMetersPerSecond * gyroAngle.getCos() + chassis.vxMetersPerSecond * gyroAngle.getSin(),
                chassis.omegaRadiansPerSecond);
    }

    
}
