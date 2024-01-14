package frc.autonomous;

import frc.robot.Drivetrain;

import java.util.HashMap;
import java.util.List;

import com.choreo.lib.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import frc.robot.Robot;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
public class ChoreoAutonomous {
    public Command getChoreoCommand() {
        ChoreoTrajectory traj;
                traj = Choreo.getTrajectory("Trajectory");

                Command swerveCommand = Choreo.choreoSwerveCommand(
                traj, // Choreo trajectory from above
                Drivetrain::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
                new PIDConstants(0.5, 0.0, 0.0), // PIDController for field-relative X
                                                // translation (input: X error in meters,
                                                // output: m/s).
                new PIDConstants(0.5, 0.0, 0.0), // PIDController for field-relative Y
                                                // translation (input: Y error in meters,
                                                // output: m/s).
                new PIDConstants(0, 0, 0.0), // PID constants to correct for rotation
                                             // error

                                             //These might be wrong, as I translated the PID constants over to replace PIDControllers
                (ChassisSpeeds speeds) -> swerve.drive( // needs to be robot-relative
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                false),
                true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
                Drivetrain // The subsystem(s) to require, typically your drive subsystem only
                );
                return Commands.sequence(
                        Commands.runOnce(() -> Robot.swerve.resetOdometry(traj.getInitialPose())),
                        swerveCommand,
                        Robot.swerve.run(() -> swerve.drive(0, 0, 0, false))
    );
    }
}
