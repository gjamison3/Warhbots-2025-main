package frc.robot.commands;

// Imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/** Drives the robot with a locked heading*/
public class LockHeadingToReef extends Command {
    // Declare variables that will be initialized by the constructor
    private final SwerveDrivetrain drivetrain;
    private final DoubleSupplier inputX;
    private final DoubleSupplier inputY;
    private final IntSupplier targetID;
    private int lastKnownHeading = 0;
    private final PIDController headingController;
    
    /** 
     * Constructs a DriveWithHeading command
     *  
     * @param drivetrain The robot's drivetrain
    */
    public LockHeadingToReef(SwerveDrivetrain drivetrain, DoubleSupplier translationInputX, DoubleSupplier translationInputY, IntSupplier targetID) {
        // Initialize internal variables with values passed through params
        this.drivetrain = drivetrain;
        this.inputX = translationInputX;
        this.inputY = translationInputY;
        this.targetID = targetID;
        this.headingController = new PIDController(.04, 0, .0005);
        
        // Tell the CommandBase that this command uses the drivetrain
        addRequirements(drivetrain);
    }

    @Override // Configure the PID controller
    public void initialize() {
        headingController.setTolerance(2); // Allow for 2 degrees of rotational error
        headingController.enableContinuousInput(-180, 180); // -180 and 180 are the same heading
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get the current values from double suppliers
        double x = inputX.getAsDouble();
        double y = inputY.getAsDouble();
        int tid = targetID.getAsInt();
        
        // Preprocess drive instructions coming from joysticks
        x = drivetrain.preprocessX(x);
        y = drivetrain.preprocessY(y);

        // Get the desired heading based on the target's ID
        // Uses fall-through behavior to implement similar behavior in disparate cases
        int desiredHeading;
        switch (tid) {
            case 18: case 7:
                desiredHeading = 0;
                break;
            case 19: case 6:
                desiredHeading = 60;
                break;
            case 20: case 11:
                desiredHeading = 120;
                break;
            case 21: case 10:
                desiredHeading = 180;
                break;
            case 22: case 9:
                desiredHeading = 240;
                break;
            case 17: case 8:
                desiredHeading = 300;
                break;
        
            default: // If we see no target, just use the same target heading we used last frame
                desiredHeading = lastKnownHeading;
        }
        
        // Calculate the pid controller's rotation output
        double rotation = headingController.calculate(desiredHeading, drivetrain.getHeadingDegrees());
               
        // Send a drive instruction to the drivetrain
        drivetrain.sendDrive(x, y, rotation, true);
    }

    // Command ends when robot is done rotating
    @Override
    public boolean isFinished() {
        return headingController.atSetpoint();
    }

    // Runs when the command ends
    @Override
    public void end(boolean interrupted) {}
}