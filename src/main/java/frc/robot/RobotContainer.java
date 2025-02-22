package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.*;

// Subsystem Imports
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
// Command imports
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.SetUpperChassisPose;
import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.wpilibj.DriverStation;

// Other imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_PORT);

    public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Pivot pivot = new Pivot();

    private final AutonContainer auton = new AutonContainer(this);
    private final SendableChooser<Command> autonChooser = auton.buildAutonChooser();

    /** Constructs a RobotContainer */
    public RobotContainer() {
        SmartDashboard.putData("Auton Selector", autonChooser);

        setDriverControls();
        setOperatorControls();
        setDefaultCommands();
    }

    /** Sets the default commands for the robot's subsystems */
    private void setDefaultCommands() {
    }
    
    /** Configures a set of control bindings for the robot's driver */
    private void setDriverControls() {
        // Drives the robot with the joysticks
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, 
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightX(),
        // 10 is placeholder for maxHeight before speed reduction
        // .5 is placeholder for slow speed, 1 is placeholder for fast speed
        () -> elevator.getPosition() > 10 ? .5 : 1));
        
        // HOLD RT -> Drive in robot centric mode
        driverController.leftTrigger()
            .onTrue(new InstantCommand(() -> drivetrain.setFieldCentric(false)))
            .onFalse(new InstantCommand(() -> drivetrain.setFieldCentric(true)));

        driverController.leftBumper().onTrue(new InstantCommand(() -> drivetrain.resetHeading()));
    }

    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {
        operatorController.x().onTrue(new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L2_SCORE));
        operatorController.b().onTrue(new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L3_SCORE));
        operatorController.y().onTrue(new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L4_SCORE));
        operatorController.a().onTrue(new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L1_SCORE));
        //operatorController.rightTrigger().whileTrue(/** Intake */);
        //operatorController.leftTrigger().whileTrue(/** Shoot */);
        operatorController.pov(0).onTrue(new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L3_REMOVE));
        operatorController.pov(180).onTrue(new SetUpperChassisPose(elevator, pivot, UpperChassisPose.L2_REMOVE));
        operatorController.pov(90).onTrue(new SetUpperChassisPose(elevator, pivot, UpperChassisPose.PROCESSOR_SCORE));
    }
        
    
    /** @return Whether the robot is on the red alliance or not */
    public boolean onRedAlliance() { 
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    /** Use this to pass the autonomous command to the main {@link Robot} class. */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
