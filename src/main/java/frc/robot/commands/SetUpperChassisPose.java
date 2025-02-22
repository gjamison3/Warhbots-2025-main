package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/** Moves the pivot to a safe position, then drives the elevator 
 *  to the correct height, then moves the pivot to its destination and ends */
public class SetUpperChassisPose extends SequentialCommandGroup{
    public SetUpperChassisPose(Elevator elevator, Pivot pivot, UpperChassisPose pose){
        addCommands(
            pivot.goToPosition(UpperChassisPose.ZERO),
            elevator.elevateToPosition(pose),
            pivot.goToPosition(pose)
        );
    }
}
