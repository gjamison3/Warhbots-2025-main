package frc.robot.commands;

import static frc.robot.Constants.SwerveConstants.PP_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.BargeFling;
import frc.robot.commands.BargeScore;



/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer {
    private SwerveDrivetrain drivetrain;
    private Pivot pivot;
    private Shooter shooter;
    private Elevator elevator;

    /** Constructs an AutonContainer object */ 
    public AutonContainer(RobotContainer robot) {
        this.drivetrain = robot.drivetrain;
        pivot = robot.pivot;
        shooter = robot.shooter;
        elevator = robot.elevator;
        registerNamedCommands();

        // Attempt to load pp settings from GUI
        // Fallback onto code defined config, it's better than nothing
        RobotConfig config = PP_CONFIG;
        try { config = RobotConfig.fromGUISettings(); }
        catch(Exception e) { e.printStackTrace(); }

        AutoBuilder.configure(
            drivetrain::getPoseMeters, 
            drivetrain::setOdometry,
            drivetrain::getChassisSpeeds,
            drivetrain::driveRobotRelative,
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> robot.onRedAlliance(),
            drivetrain);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("doNothing", new DoNothing(15.0, drivetrain) );
        NamedCommands.registerCommand("elevatorZero", new SetUpperChassisPose(elevator, pivot, UpperChassisPose.ZERO));
        NamedCommands.registerCommand("elevatorL2", new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L2_SCORE));
        NamedCommands.registerCommand("elevatorL3", new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L3_SCORE));
        NamedCommands.registerCommand("elevatorL4", new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L4_SCORE));
        NamedCommands.registerCommand("elevatorAlgaeL2", new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L2_REMOVE));
        NamedCommands.registerCommand("elevatorAlgaeL3", new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L3_REMOVE));
        NamedCommands.registerCommand("elevatorBarge", new SetUpperChassisPose(elevator, pivot, UpperChassisPose.BARGE_SETUP));
        NamedCommands.registerCommand("bargeFling", new BargeFling(shooter, pivot));
        NamedCommands.registerCommand("shoot", shooter.shoot(.5).withTimeout(.5));
        NamedCommands.registerCommand("algaeIntake", shooter.shoot(-.5).withTimeout(.5));
    }

    public SendableChooser<Command> buildAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("Do Nothing", new DoNothing(0, drivetrain));
        //chooser.addOption("Test Spin", AutoBuilder.buildAuto("Example Spin"));
        chooser.addOption("Test Straight", AutoBuilder.buildAuto("Straight"));
        chooser.addOption("Move back and scoreL4", AutoBuilder.buildAuto("Move back and scoreL4"));
        //chooser.addOption("Move back and scoreL4 then intake", AutoBuilder.buildAuto("Move back and scoreL4 then intake"));
        chooser.addOption("new new test 2", AutoBuilder.buildAuto("new new test 2"));
        chooser.addOption("new new test 3", AutoBuilder.buildAuto("new new test 3"));
        chooser.addOption("new new test 4", AutoBuilder.buildAuto("new new test 4"));
        return chooser;
    }
}