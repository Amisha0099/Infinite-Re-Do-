package org.team3128.bluepercents.subsystems;

import org.team3128.common.utility.units.Length;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.test_suite.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.team3128.common.utility.datatypes.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.RobotConstants;

// the superclass is purely for semantic purposes
public class Constants extends RobotConstants {


        public static class MechanismConstants {
                public static final double ENCODER_RESOLUTION_PER_ROTATION = 2048;
                public static final double inchesToMeters = 0.0254;
                public static final double DT = 0.005; // time between update() method calls for mechanisms
        }

        public static class DriveConstants {
                public static final double kDriveInchesPerSecPerNUp100ms = (1000d / 1)
                                * (1 / MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                                * (Constants.DriveConstants.WHEEL_DIAMETER * Math.PI)
                                * Constants.DriveConstants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION; // a fairly basic relationship between tangential and rotational speed: NU/100ms * 1000ms/1second * 1/(encoder resolution) * CIRCUM * (relation between encoder rotations and wheel rotations) = in/s
                public static final double kDriveNuToInches = (1
                                / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                                * Constants.DriveConstants.WHEEL_DIAMETER * Math.PI
                                * Constants.DriveConstants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION;

                public static final NeutralMode DRIVE_IDLE_MODE = NeutralMode.Brake;

                public static final double ENCODER_ROTATIONS_FOR_ONE_WHEEL_ROTATION = 72 / 8; // basically your gearing. Ask Mech for gear teeth number to gear teeth number ratio: 8.3333333

                public static final double WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION = 1
                                / Constants.DriveConstants.ENCODER_ROTATIONS_FOR_ONE_WHEEL_ROTATION;

                public static final int RIGHT_DRIVE_FRONT_ID = 3;
                public static final int RIGHT_DRIVE_MIDDLE_ID = 2;

                public static final int LEFT_DRIVE_FRONT_ID = 1;
                public static final int LEFT_DRIVE_MIDDLE_ID = 0;

                public static final int DRIVE_HIGH_SPEED = 140; // Empirical Max Linear Speed: TBD in/s

                public static final double WHEEL_DIAMETER = 3.55; // effective wheel diameter (measure first then tune this number until distances are accurate)

                public static final double LEFT_SPEEDSCALAR = 1.0; // purely for TELEOP drive (to make sure that when the drive pushes the joystick forward, both sides of the drivetrain are going ROUGHLY the same speed)
                public static final double RIGHT_SPEEDSCALAR = 1.0;// purely for TELEOP drive (to make sure that when the drive pushes the joystick forward, both sides of the drivetrain are going ROUGHLY the same speed)

                public static final double DRIVE_ACCEL_LIMIT = 120; // Ballpark estimates from mech (Be conservative unless you really need the quick auto paths)
                public static final double DRIVE_JERK_LIMIT = 2000; // Ballpark estimates (Be conservative)

                public static double K_AUTO_RIGHT_P = 0.00007; // 0.00065
                public static double K_AUTO_RIGHT_D = 0.000;
                public static double K_AUTO_RIGHT_F = 1 / 145.9150145782 * kDriveInchesPerSecPerNUp100ms; // 1/(consistent max vel of this side of drivetrain in/s) * conversion to NU/s
                public static double K_AUTO_LEFT_P = 0.00007;
                public static double K_AUTO_LEFT_D = 0.000; // 0.0001
                public static double K_AUTO_LEFT_F = 1 / 140.8705712261 * kDriveInchesPerSecPerNUp100ms; // 1/(consistent max vel of this side of drivetrain in/s) * conversion to NU/s
                public static final double K_HOLD_P = 4;

                public static final double kS = 0.178;
                public static final double kV = 0.055;//0.0516;
                public static final double kA = 0.00679;
                public static final double kP = 0.0013;

        }

        public static class AutonomousDriveConstants {
                public static final double TRACK_RADIUS = 23.89231390411386;
                public static final double MIN_TURNING_RADIUS = 40;
                public static final double MIN_PATH_SPEED = 20;
                public static final double MAX_PATH_SPEED = 120;
                public static final double MIN_LOOKAHEAD_DISTANCE = 14;
                public static final double MAX_LOOKAHEAD_DISTANCE = 30;
                public static final double MAX_TURN_ERROR = 2;
                public static final double MAX_PID_STOP_SPEED = 8;
        }

        public static class VisionConstants {
                public static final double BOTTOM_LIMELIGHT_HEIGHT = 28.85 * Length.in;
                public static final double PIVOT_HEIGHT = 16.0;
                public static final double BOTTOM_LIMELIGHT_ANGLE = 30.0 * Angle.DEGREES;
                public static final double BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT = 1 * Length.in;
                public static final int SAMPLE_RATE = 3;
                public static final double TX_THRESHOLD = 2; // the maximum error in tx where the shooter will be allowed to shoot
                public static final double TX_OFFSET = 0; // to offset alignment in either direction
                public static final PIDConstants VISION_PID = new PIDConstants(0, 0.014, 0.02, 0.00006);
        }


        public static class BoxIntakeConstants {
                
                public static final int LEFT_BOX_MOTOR_ID = 10;
                public static final int RIGHT_BOX_MOTOR_ID = 9;

                public static final int SENSOR_BOX_ID = 8;

                public static final double BOX_MOTOR_ON_VALUE = -0.65;
                public static final double BOX_MOTOR_OFF_VALUE = 0;

                public static final double BOX_OFFSET_VALUE = 2;
                
        }

        public static class BellIntakeConstants {
                public static final int BELL_MOTOR_ID = 14;

                public static final int SENSOR_BELL_ID = 15;
                public static final double BELL_MOTOR_ON_VALUE = -0.5;
                public static final double BELL_MOTOR_OFF_VALUE = 0;

                public static final double BELL_OFFSET_VALUE = 1;
        }

        
        public static class ClimberConstants {
                public static final int LEFT_CLIMBER_MOTOR_ID = 12;
                public static final int RIGHT_CLIMBER_MOTOR_ID = 13;
                public static final int TURNING_CLIMBER_MOTOR_ID = 14;
                public static final double CLIMB_POWER = 0.5;

                public static final PIDConstants CLIMBER_PID = new PIDConstants(0, 0.15, 0.01, 0);
                public static final double CLIMBER_SATURATION_LIMIT = 3 / CLIMBER_PID.kI;
        }

        public static class ArmConstants {
                public static final int LOWER_MOTOR_ID = 13;
                public static final int UPPER_MOTOR_ID = 4;

                public static final NeutralMode ARM_NEUTRAL_MODE = NeutralMode.Brake;
                public static final NeutralMode ARM_NEUTRAL_MODE_DEBUG = NeutralMode.Coast;
        
                public static final double ARM_GEARING = 60 / 12 * 80 / 18 * 64 / 8; // for every (ARM_GEARING)
                                                                                     // rotations of the
                // motor, we get 1 rotation of the arm (ask
                // mech for this)

                public static final double MAX_ARM_ANGLE = 180;
                public static final double LIMELIGHT_ARM_LENGTH = 19;
                public static final double LIMELIGHT_ARM_ANGLE = 40.26;

                public static final PIDConstants ARM_PID = new PIDConstants(0, 0.15, 0.01, 0);
                public static final double kf_term = 0.12;

                public static final double ARM_SATURATION_LIMIT = 2 / ARM_PID.kI; // set limit on integral accumulation

                public static final double STARTING_ANGLE = 48.87; //the angle that the arm is at when it is within frame perimeter
                public static final int STARTING_POSITION = (int) ((STARTING_ANGLE / 360) * MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION * ARM_GEARING);

                public static final double g = 9.81;
                public static final double LOWER_MASS = 40;
                public static final double LOWER_RADIUS = 20;

                public static final double UPPER_BOX_MASS = 30;
                public static final double UPPER_BELL_MASS = 15;
                public static final double UPPER_BOX_RADIUS = 10;
                public static final double UPPER_BELL_RADIUS = 8;

                public static final double SPIN_WHEEL_POWER = 15;
        }

}