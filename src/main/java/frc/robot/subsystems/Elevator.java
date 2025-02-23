package frc.robot.subsystems;

import static frc.robot.Constants.SubsystemIDs.ELEVATOR_LEFT_MOTOR;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_RIGHT_MOTOR;
import static frc.robot.Constants.UpperChassisConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private SparkMax leader;
    private SparkMax follower;
    private ProfiledPIDController pidController;
    private UpperChassisPose targetPosition = UpperChassisPose.ZERO;

    public Elevator() {
        leader = new SparkMax(ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        follower = new SparkMax(ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);
        pidController = new ProfiledPIDController(ELEVATOR_P, 0, ELEVATOR_D,
            new TrapezoidProfile.Constraints(ELEVATOR_VEL_LIMIT, ELEVATOR_ACCEL_LIMIT));
        pidController.setTolerance(1);
        
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.encoder.positionConversionFactor(ELEVATOR_RATIO);
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(leader, false);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** @return A command that tells the elevator to go to a position */
    public Command elevateToPosition(UpperChassisPose position) {
        return run(() -> driveToPosition(position))
            .until(() -> atSetpoint());
    }

    
    // Getters
    public double getPosition() { return leader.getEncoder().getPosition(); }
    public double getTargetPosition() { return targetPosition.getElevatorHeight(); }
    public boolean atSetpoint() { return pidController.atSetpoint(); }

    /** Sends voltage to the elevator to drive it to a position */
    public void driveToPosition(UpperChassisPose position) {
        targetPosition = position;
        double pidout = pidController.calculate(getPosition(), position.getElevatorHeight());
        leader.setVoltage(pidout * RobotController.getBatteryVoltage());
    }   


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Target", targetPosition.getElevatorHeight());
        SmartDashboard.putBoolean("Elevator at Target", atSetpoint());
    }
}
