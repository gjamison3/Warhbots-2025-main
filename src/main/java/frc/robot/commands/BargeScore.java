package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

/** Moves the pivot to a safe position, then drives the elevator 
 *  to the correct height, then moves the pivot to its destination and ends */
public class BargeScore extends SequentialCommandGroup{
    public BargeScore(Elevator elevator, Pivot pivot, Shooter shooter, UpperChassisPose pose){
        addCommands(
            pivot.goToPosition(UpperChassisPose.BARGE_SETUP),
            elevator.elevateToPosition(pose),
            new WaitCommand(2.0),
            pivot.goToPosition(pose),
            new WaitCommand(2),
            shooter.bargealgaescore(1.0),
            new WaitCommand(2),
            shooter.bargealgaescore(0)

        );
    }
}
