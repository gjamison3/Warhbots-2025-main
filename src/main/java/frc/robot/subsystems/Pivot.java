package frc.robot.subsystems;

import static frc.robot.Constants.UpperChassisConstants.*;
import static frc.robot.Constants.SubsystemIDs.PIVOT_MOTOR;

import frc.robot.Constants.UpperChassisConstants.UpperChassisPose;
import frc.robot.wrappers.GenericPID;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Pivot extends SubsystemBase {
    private SparkMax motor;
    private GenericPID pidController;
    private UpperChassisPose targetPosition;

    public Pivot() {
        motor = new SparkMax(PIVOT_MOTOR, MotorType.kBrushless);
        pidController = new GenericPID(motor, ControlType.kPosition, PIVOT_P);
        pidController.setInputRange(PIVOT_MIN, PIVOT_MAX);

        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(PIVOT_RATIO);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public Command goToPosition(UpperChassisPose position) {
        targetPosition = position;
        return run(() -> pidController.activate(position.getPivotAngle()))
            .until(() -> atSetpoint());
    }


    // Getters
    public boolean atSetpoint() { return pidController.atSetpoint(1); }
    public double getPosition() { return pidController.getMeasurement(); }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Position", getPosition());
        SmartDashboard.putNumber("Shooter Target", targetPosition.getPivotAngle());
        SmartDashboard.putBoolean("Shooter at Target", atSetpoint());
    }
}
