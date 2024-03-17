package frc.autonomous;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.amoghbelt.ShootNote;
import frc.commands.shooter.DynamicShootSpeaker;
import frc.commands.shooter.SetShooterAmp;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;


import static frc.autonomous.AutonomousProgram.create;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * Quick guide to Comand Groups:
 *
 * SequentialComandGroup:
 * Will run all comands in order within it's parentheses
 * Note: If a comand does not have a isFinshed statment the code will be stuck
 * on that command foreverk
 *
 * ParallelCommandGroup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Both commands will have to finish to move on
 *
 * ParallelRaceGoup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: As soon as one command runs it's isfinished method runs then both
 * commands will end
 *
 * ParallelDeadlineGroup
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Only the first command will finish the group
 */
public class Autonomous {
    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    // global event map

    // This is just an example event map. It would be better to have a constant,
    // in your code that will be used by all path following commands.

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.

    public static void init() {
        // create("Test", () -> new InstantCommand(() -> System.out.println("TEST")));
        // create("Alan is a persecuter", () -> Robot.swerve.getAutonomousCommand("Alan
        // is a persecuter"));
        // create("Alan is not a persecuter", () ->
        // Robot.swerve.getAutonomousCommand("Alan is not a persecuter"));
        create("test",
                () -> new ParallelDeadlineGroup(Commands.waitSeconds(2), new SetShooterAmp(Math.toRadians(45), 2)));
        create("IndexerShootLeft", () -> new SequentialCommandGroup(
                new ParallelDeadlineGroup(Commands.waitSeconds(2), new ShootNote(),
                        new SetShooterAmp(50,21).repeatedly()),
                Robot.swerve.getAutonomousCommand("Taxileft")));
        create("IndexerShootMid", () -> new SequentialCommandGroup(
                new ParallelDeadlineGroup(Commands.waitSeconds(2), new ShootNote(),
                        new SetShooterAmp(50,21.0).repeatedly()),
                Robot.swerve.getAutonomousCommand("Taxicenter")));
        create("IndexerShootRight", () -> new SequentialCommandGroup(
                new ParallelDeadlineGroup(Commands.waitSeconds(2), new ShootNote(),
                        new SetShooterAmp(50,21).repeatedly()),
                Robot.swerve.getAutonomousCommand("Taxiright")));
        create("MessUpAmp", () -> Robot.swerve.getAutonomousCommand("MessUpAmp"));
        create("MessUpMiddle", () -> Robot.swerve.getAutonomousCommand("MessUpMiddle"));
        create("MessUpStage", () -> Robot.swerve.getAutonomousCommand("MessUpStage"));
        create("ShootTaxi", () -> Robot.swerve.getAutonomousCommand("ShootTaxi"));
        create("AutoMoveTest", () -> Robot.swerve.getAutonomousCommand("AutoMoveTest"));
        create("Taxileft",() -> Robot.swerve.getAutonomousCommand("Taxileft"));
        create("Taximid",() -> Robot.swerve.getAutonomousCommand("Taxicenter"));
        create("Taxiright",() -> Robot.swerve.getAutonomousCommand("Taxiright"));
    }
}