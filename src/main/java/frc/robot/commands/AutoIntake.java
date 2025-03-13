package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends Command {
    private Shooter shooter;
    private IntakeState state;

    private boolean entry;

    /** A list of the states the intake can be in */
    public enum IntakeState {
        /** Sets the intake to a fast speed, ideal for getting a good hold on a piece */
        WAITING(.3),
        /** Sets the intake to a slow speed, ideal for indexing a piece */
        INDEXING(.13),
        /** Stops the intake, as to not overshoot the piece */
        HOLDING(0);

        public double speed;
        private IntakeState(double speed) { this.speed = speed; }
    }


    /** Constructs an AutoIntake Command
     * This command attempts to index pieces as they come into the intake.
     * It is meant to be run as a default command for the system, and cancelled
     * by another command requiring the subsystem when pieces are shot.
     * This is important because the cancellation should trigger a rerun of the
     * initialize function.
     * 
     * @param shooter The intake/shooter system used to index pieces.
     */
    public AutoIntake(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }


    @Override
    public void initialize() {
        pollSensors();

        // Start in the waiting state
        enterState(IntakeState.WAITING);        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pollSensors();
        runStateMachine();

        SmartDashboard.putString("Intake State", state.toString());
    }

    private void pollSensors() {
        entry = shooter.isSensorBlocked();
    }

    /** Runs the state machine by evaluating whether we should leave
     *  our current state, and which state we should transition to.
     */
    private void runStateMachine() {
        switch (state) {
            case WAITING: // Once the piece is seen, slow down
                if(entry) { enterState(IntakeState.INDEXING); }
                break;

            case INDEXING: // Once the piece is beyond the sensor, just hold it still
                if(!entry) { enterState(IntakeState.HOLDING); }
                break;

            case HOLDING: // Never leave
                break;
        
            // If in an invalid state, transition to done
            default: enterState(IntakeState.HOLDING);
        }
    }

    private void enterState(IntakeState destination) {
        shooter.spin(destination.speed);
        state = destination;
    }
}
