package frc.robot.subsystems;

import static frc.robot.Constants.SubsystemIDs.CLIMBER_MOTOR;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class Climber extends SubsystemBase {
    private SparkMax motor;
    

    public Climber() {
        motor = new SparkMax(CLIMBER_MOTOR, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(40);
        motorConfig.idleMode(IdleMode.kBrake);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    }

    
    public Command climbin(double speed) {
        return runEnd(() -> motor.set(speed),
            () -> motor.set(0));
    }

    public Command climbout(double speed) {
        return runEnd(() -> motor.set(-speed),
            () -> motor.set(0));
    }

    

}
