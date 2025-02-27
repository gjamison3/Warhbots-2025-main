package frc.robot.subsystems;

import static frc.robot.Constants.SubsystemIDs.ELEVATOR_LEFT_MOTOR;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_RIGHT_MOTOR;
import static frc.robot.Constants.UpperChassisConstants.*;

import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Elevator extends SubsystemBase {
    // Declare variables
    public SparkMax leadMotor;
    public SparkMax followerMotor; 
    public ProfiledPIDController elevatorPID;
    public UpperChassisPose lastPosition = UpperChassisPose.ZERO;
    

    // Constructor 
    public Elevator() {
        leadMotor = new SparkMax(ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        followerMotor = new SparkMax(ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);
        elevatorPID = new ProfiledPIDController(ELEVATOR_P, 0, ELEVATOR_D, 
            new Constraints(ELEVATOR_VEL_LIMIT, ELEVATOR_ACCEL_LIMIT));
        elevatorPID.setTolerance(1);
        
        // Configure the elevator motors   
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.encoder.positionConversionFactor(ELEVATOR_RATIO);
        leaderConfig.smartCurrentLimit(40);
        leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(leadMotor, false);
        followConfig.idleMode(IdleMode.kCoast);
        followConfig.smartCurrentLimit(40);
        followerMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
    }

    
    /** @return A command that tells the elevator to go to a position */
    public Command elevateToPosition(UpperChassisPose pos) {
        return new WaitCommand(.1)
            .alongWith(this.runOnce(() -> elevatorPID.setGoal(pos.getElevatorHeight())))
            .andThen(
                run(() -> {})
                .until(() -> atSetpoint())
                );
    }

    // Getters
    public double getPosition() { return leadMotor.getEncoder().getPosition(); }
    public boolean atSetpoint() { return elevatorPID.atSetpoint(); }
    public UpperChassisPose getLatestPosition() { return lastPosition; }

    /** Sends voltage to the elevator to drive it to a position */
    private void driveElevatorToPosition() {
        double pidout = elevatorPID.calculate(leadMotor.getEncoder().getPosition());
        leadMotor.setVoltage(pidout * RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {
        driveElevatorToPosition();

        SmartDashboard.putNumber("Elevator Height", getPosition());
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        SmartDashboard.putString("Elevator Position", lastPosition.toString());
    }
}