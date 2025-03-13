package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

/** Moves the pivot to a safe position, then drives the elevator 
 *  to the correct height, then moves the pivot to its destination and ends */
public class BargeFling extends ParallelCommandGroup{
    public BargeFling(Shooter shooter, Pivot pivot){
        addCommands(
            pivot.goToPosition(UpperChassisPose.BARGE_SCORE),
            new SequentialCommandGroup(new WaitCommand(0.125), shooter.bargealgaescore(0.5).withTimeout(0.5))
        );
    }
}
