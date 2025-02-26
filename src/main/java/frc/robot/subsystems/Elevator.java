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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    // Declare variables
    public SparkMax leadMotor;
    public SparkMax followerMotor; 
    public ProfiledPIDController elevatorPID;
    public UpperChassisPose target = UpperChassisPose.ZERO;
    

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
        leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(leadMotor, false)
            .idleMode(IdleMode.kCoast);
        followerMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
    }


    // Getters
    public UpperChassisPose getTargetPosition() { return target; }
    public double getHeight() { return leadMotor.getEncoder().getPosition(); }
    public double getSetpoint() { return elevatorPID.getGoal().position; }
    public boolean atSetpoint() { return elevatorPID.atSetpoint(); }

    /** Sets the position the elevator will drive to */
    public void setPosition(UpperChassisPose position) { 
        target = position;
        elevatorPID.setGoal(position.getElevatorHeight()); 
    }

    /** Sends voltage to the elevator to drive it to the current target position */
    private void driveElevator() {
        double pidout = elevatorPID.calculate(getHeight());
        leadMotor.setVoltage(pidout * RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {
        driveElevator();

        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        SmartDashboard.putString("Elevator Position", target.toString());
    }
}