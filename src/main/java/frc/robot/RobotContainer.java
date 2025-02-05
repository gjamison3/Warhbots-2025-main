package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.*;

// Subsystem Imports
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

// Command imports
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.wpilibj.DriverStation;

// Other imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_PORT);

    public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

    private final AutonContainer auton = new AutonContainer(this);
    private final SendableChooser<Command> autonChooser = auton.buildAutonChooser();

    /** Constructs a RobotContainer */
    public RobotContainer() {
        SmartDashboard.putData("Auton Selector", autonChooser);

        setDriverControls();
        setOperatorControls();
    }

    
    /** Configures a set of control bindings for the robot's driver */
    private void setDriverControls() {
        // Drives the robot with the joysticks
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, 
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightX()));
        
        // HOLD RT -> Drive in robot centric mode
        driverController.rightTrigger().onTrue(new InstantCommand(() -> drivetrain.setFieldCentric(false)))
        .onFalse(new InstantCommand(() -> drivetrain.setFieldCentric(true)));
    }

    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {
        // Runs the auton command as an example binding
        operatorController.a().whileTrue(getAutonomousCommand());
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
