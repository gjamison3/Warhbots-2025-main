package frc.robot;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

// Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/* 
 * To access numbers in this file, import or statically import one of its subclasses:
 * example:
 * import static frc.robot.Constants.ControllerPorts.*;
 * import frc.robot.Constants.DriveConstants;
 */
public final class Constants {

    /* -------------- IDs -------------- */

    /** Ports used by controllers. */
    public static final class ControllerPorts {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class SubsystemIDs {
        public static final int ELEVATOR_LEFT_MOTOR = 10;
        public static final int ELEVATOR_RIGHT_MOTOR = 11;
        public static final int PIVOT_MOTOR = 12;
        public static final int INTAKE_MOTOR = 13;
        public static final int INTAKE_SENSOR = 23;
        public static final int CLIMBER_MOTOR = 14;
    }

    /** IDs used by the swerve drivetrain.
        3X for turning, 4X for driving, 5X for abs encoders. */
    public static final class SwerveIDs {
        // Front left module
        public static final int FL_TURN_ID = 30;
        public static final int FL_DRIVE_ID = 40;
        public static final int FL_ENCODER_ID = 50;
        // Front right module
        public static final int FR_TURN_ID = 31;
        public static final int FR_DRIVE_ID = 41;
        public static final int FR_ENCODER_ID = 51;
        // Rear left module
        public static final int RL_TURN_ID = 32;
        public static final int RL_DRIVE_ID = 42;
        public static final int RL_ENCODER_ID = 52;
        // Rear right module
        public static final int RR_TURN_ID = 33;
        public static final int RR_DRIVE_ID = 43;
        public static final int RR_ENCODER_ID = 53;
    }

    /* -------------- SUBSYTEM CONSTANTS -------------- */

    public static final class UpperChassisConstants {
        public static final double ELEVATOR_RATIO = 1;
        public static final double ELEVATOR_VEL_LIMIT = 100;
        public static final double ELEVATOR_ACCEL_LIMIT = 40;
        public static final double ELEVATOR_P = 0.15;
        public static final double ELEVATOR_D = 0.005;

        public static final double PIVOT_RATIO = 1;
        public static final double PIVOT_P = 0.09;
        public static final double PIVOT_MIN = 0;
        public static final double PIVOT_MAX = 25;

        public enum UpperChassisPose {
            /** It is assumed that the elevator is safe to move when the pivot is at 0 */
            ZERO(0, 0.0),
            L1_SCORE(0, 3.3),
            L2_SCORE(5.92, 3.3),
            L3_SCORE(14.92, 3.3),
            L4_SCORE(32.0, 10.0),
            L2_REMOVE(12.10, 21.93),
            L3_REMOVE(20.23, 21.93),
            BARGE_SETUP(33.5,21.93),
            BARGE_SCORE(33.5,7.0),
            PROCESSOR_SCORE(1.5, 21.93);

            private final double elevatorHeight;
            private final double pivotAngle;
            UpperChassisPose(double height, double angle) {
                this.elevatorHeight = height;
                this.pivotAngle = angle;
            }
            public double getElevatorHeight() { return elevatorHeight; }
            public double getPivotAngle() { return pivotAngle; }
        }
    }

    /** Turning a module to absolute 0 minus its offset will point it forward */
    public static final class SwerveModuleOffsets {
        public static final double FL_OFFSET = -76.729;
        public static final double FR_OFFSET = 68.6426;
        public static final double RL_OFFSET = 134.5605;
        public static final double RR_OFFSET = 36.6504;
    }

    /** Whether or not each swerve component should be inverted/reversed */
    public static final class SwerveInversions {
        // Whether each driving motor should be inverted
        public static final boolean INVERT_FL_DRIVE = false;
        public static final boolean INVERT_FR_DRIVE = true;
        public static final boolean INVERT_RL_DRIVE = false;
        public static final boolean INVERT_RR_DRIVE = true;

        // Whether each turning motor should be inverted
        public static final boolean INVERT_FL_TURN = true;
        public static final boolean INVERT_FR_TURN = true;
        public static final boolean INVERT_RL_TURN = true;
        public static final boolean INVERT_RR_TURN = true;
    }

    /** Constants related to swerve calculations */
    public static final class SwerveConstants {
        /** The distance between the left and right wheels in inches */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.75);
        /** The distance between the front and rear wheels in inches */
        public static final double WHEEL_BASE = Units.inchesToMeters(22.75);

        /** An array containing the position of each module as a {@link Translation2d} object */
        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),  // FR
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // RR
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),   // FL
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),   // RL
        };

        // Kinematics are used to calculate how each module needs to move
        // in order to move the robot as a whole in a certain way

        /** Standard kinematics with center of rotation located at the center of the robot */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        /** The max speed the robot is allowed to drive in m/sec */
        public static final double MAX_TRANSLATION_SPEED = 5.06;
        /** The max speed the robot is allowed to spin in rads/sec */
        public static final double MAX_ROTATION_SPEED = Math.PI;
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_ROTATION_SPEED, Math.PI * 2);

        public static final class ModuleConstants {
            /** The ratio of the drive motors on the workhorse chassis */
            public static final double DRIVE_RATIO_SLOW = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));
            /** The ratio of the drive motors on the robot */
            public static final double DRIVE_RATIO_FAST = 1 / ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0));
            /** The ratio of the turning motors */
            public static final double TURN_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
            
            /** The diameter of the wheels measured in meters */
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.96);

            /** Drive motor revolutions * DRIVE_REVS_TO_M = distance in meters */
            public static final double DRIVE_REVS_TO_M = ((WHEEL_DIAMETER * Math.PI) / DRIVE_RATIO_FAST);

            // 1 RPM * DRIVE_REVS_TO_M = speed in m/min. Divide by 60 to find m/sec
            /** Drive motor RPM * DRIVE_RPM_TO_MPS = speed in m/sec */
            public static final double DRIVE_RPM_TO_MPS = DRIVE_REVS_TO_M / 60.0;

            /** Turning motor revolutions * TURNING_REVS_TO_DEG = Turning motor total degrees turned */
            public static final double TURNING_REVS_TO_DEG =  360.0 / TURN_RATIO;
        }
    
        /** Enum representing the four possible positions a module can occupy */
        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }

        // Values with 0 are unknown and should be replaced with the proper values for your robot
        private static final DCMotor PP_DRIVE_MOTOR = new DCMotor(12.6, 0, 0, 0, 0, 1);
        private static final ModuleConfig PP_MODULE_CONFIG = new ModuleConfig(ModuleConstants.WHEEL_DIAMETER / 2, MAX_TRANSLATION_SPEED, 1, PP_DRIVE_MOTOR, 40, 1);
        public static final RobotConfig PP_CONFIG = new RobotConfig(0, 0, PP_MODULE_CONFIG, MODULE_TRANSLATIONS);
    }

    public static final class DriveConstants {
            /** Higher values make the robot drive more aggressively */
            public static final double TRANSLATION_SLEW = 4;
            /** Higher values make the robot spin more aggressively */
            public static final double ROTATION_SLEW = 6;
    
            /** Translation instructions closer to 0 than the deadband will be set to 0 */
            public static final double TRANSLATION_DEADBAND = .05;
            /** Rotation instructions closer to 0 than the deadband will be set to 0 */
            public static final double ROTATION_DEADBAND = .1;
    }
}
