package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import edu.wpi.first.wpilibj.TimedRobot;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonCommand;

    @Override
    public void robotInit() {
        // Start the camera feed
        CameraServer.startAutomaticCapture();

        // Construct the robot container
        robotContainer = new RobotContainer();

        // Cancel any commands that may have persisted through power off or redeploy
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousInit() {
        // Get the command to be used in auton
        autonCommand = robotContainer.getAutonomousCommand();
        // Schedule the command if there is one
        if (autonCommand != null)
            autonCommand.schedule();
    }

    @Override
    public void teleopInit() {
        // Cancel the auton command when teleop starts
        CommandScheduler.getInstance().cancelAll();
        if (autonCommand != null)
            autonCommand.cancel();
    }

    @Override
    public void robotPeriodic() {    
        // Run the command scheduler always
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledExit() {
        // Reset the elevator to zero on enable
        robotContainer.elevator.setPosition(UpperChassisPose.ZERO);
    }
}