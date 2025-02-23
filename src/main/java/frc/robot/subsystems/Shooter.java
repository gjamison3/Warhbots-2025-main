package frc.robot.subsystems;

import static frc.robot.Constants.SubsystemIDs.INTAKE_MOTOR;
import static frc.robot.Constants.SubsystemIDs.INTAKE_SENSOR;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Shooter extends SubsystemBase {
    private SparkMax motor;
    private CANrange sensor;

    public Shooter() {
        motor = new SparkMax(INTAKE_MOTOR, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sensor = new CANrange(INTAKE_SENSOR);
    }

    
    public Command intake(double speed) { // Need to calibrate detection distance
        return runEnd(() -> motor.set(speed),
            () -> motor.set(0))
            .until(() -> sensor.getDistance().getValueAsDouble() < 0/* distance */);
    }

    public Command shoot(double speed) { // Need to calibrate detection distance
        return runEnd(() -> motor.set(speed),
            () -> motor.set(0));
    }
}
