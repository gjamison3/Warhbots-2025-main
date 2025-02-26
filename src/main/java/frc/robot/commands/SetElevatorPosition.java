package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends Command {
    Elevator elevator;
    UpperChassisPose pose;

    public SetElevatorPosition(Elevator elevator, UpperChassisPose pose) {
        this.elevator = elevator;
        this.pose = pose;

        addRequirements(elevator);
    }

    
    @Override // Tell the elevator where to go
    public void initialize() {
        elevator.setPosition(pose);
    }
  
    @Override // End the command when the elevator is done moving
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
