package org.team3128.bluepercents.subsystems;

import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;

public class Arm extends Threaded {
    public static enum ArmState {
        STARTING(35, 55),
        INTAKEBELL(65, 30),
        OUTTAKEBELL(15, 75),
        INTAKEBOX(32, 58),
        OUTTAKEBOX(48, 42),
        SPINWHEEL(65, 110),
        SKEWERING(35, 130);

        public double lowerAngle;
        public double upperAngle;

        private ArmState(double lowerAngle, double upperAngle) {
            this.lowerAngle = lowerAngle;
            this.upperAngle = upperAngle;
        }
    }

    public static final Arm instance = new Arm();
    public LazyTalonFX LOWER_MOTOR, UPPER_MOTOR;
    public double lowerSetpoint, upperSetpoint;
    
    public ArmState ARM_STATE;
    boolean isZeroing = false;
    public int plateauCount = 0;

    double lowCurrent = 0;
    double lowError = 0;
    public double lowOutput = 0;
    double lowAccumulator = 0;
    double lowPrevError = 0;

    double upperCurrent = 0;
    double upperError = 0;
    public double upperOutput = 0;
    double upperAccumulator = 0;
    double upperPrevError = 0;


    public static Arm getInstance() {
        return instance;
    }

    private Arm() {
        configMotors();
        setState(ArmState.STARTING);
    }

    private void configMotors() {
        LOWER_MOTOR = new LazyTalonFX(Constants.ArmConstants.LOWER_MOTOR_ID);
        UPPER_MOTOR = new LazyTalonFX(Constants.ArmConstants.UPPER_MOTOR_ID);

        LOWER_MOTOR.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        UPPER_MOTOR.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
    }

    private void setSetpoint(double desiredLowerPos, double desiredUpperPos) {
        lowerSetpoint = desiredLowerPos;
        upperSetpoint = desiredUpperPos;
    }

    public void setState(ArmState armState) {
        ARM_STATE = armState;
        setSetpoint(armState.lowerAngle, armState.upperAngle);
    }

    public double lowerArmFeedForward(double desired) {
        double ff = Constants.ArmConstants.kf_term * Constants.ArmConstants.LOWER_MASS * Constants.ArmConstants.LOWER_RADIUS * 
                    Constants.ArmConstants.g * RobotMath.cos(desired); // ff = k * F * cos(theta) with F = m * g
        return ff;
    }
    
    public double upperArmFeedForward(double desired){
        double ff = Constants.ArmConstants.kf_term * Constants.ArmConstants.g * RobotMath.cos(desired) * 
        (Constants.ArmConstants.UPPER_BOX_MASS * Constants.ArmConstants.UPPER_BOX_RADIUS 
        - Constants.ArmConstants.UPPER_BELL_MASS * Constants.ArmConstants.UPPER_BELL_RADIUS);
        return ff;
    }

    public double getLowAngle() {
        return (((getLowEncoderPos() / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                / Constants.ArmConstants.ARM_GEARING) * 360) % 360; 
    }

    public double getHighAngle(){
        return (((getHighEncoderPos() / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                / Constants.ArmConstants.ARM_GEARING) * 360) % 360; 
    }

    public double getLowEncoderPos() {
        return LOWER_MOTOR.getSelectedSensorPosition(0);
    }

    public double getHighEncoderPos() {
        return UPPER_MOTOR.getSelectedSensorPosition(0);
    }

    public void spinWheel(){
        setState(ArmState.SPINWHEEL);
        UPPER_MOTOR.set(ControlMode.PercentOutput, Constants.ArmConstants.SPIN_WHEEL_POWER);

    }

    /*private double getEncoderVel() {
        return ARM_MOTOR_LEADER.getSelectedSensorVelocity(0);
    }
    */


    /*public boolean isReady() {
        return (plateauCount > Constants.ArmConstants.PLATEAU_THRESHOLD); 
    }
    */

    @Override
    public void update() {         
      pidLowPosition();
      pidHighPosition();
    }

    public void pidLowPosition(){
       
        if (lowerSetpoint > Constants.ArmConstants.MAX_ARM_ANGLE) {
            lowerSetpoint = Constants.ArmConstants.MAX_ARM_ANGLE;
        }

        if (lowerSetpoint < 0) {
            lowerSetpoint = 0;
        }
        
        lowCurrent = getLowAngle();
        lowError = lowerSetpoint - lowCurrent;
        lowAccumulator += lowError * Constants.MechanismConstants.DT;
        if (lowAccumulator > Constants.ArmConstants.ARM_SATURATION_LIMIT) {
            lowAccumulator = Constants.ArmConstants.ARM_SATURATION_LIMIT;
        } else if (lowAccumulator < -Constants.ArmConstants.ARM_SATURATION_LIMIT) {
            lowAccumulator = -Constants.ArmConstants.ARM_SATURATION_LIMIT;
        }
        double kP_term = Constants.ArmConstants.ARM_PID.kP * lowError;
        double kI_term = Constants.ArmConstants.ARM_PID.kI * lowAccumulator;
        double kD_term = Constants.ArmConstants.ARM_PID.kD * (lowError - lowPrevError) / Constants.MechanismConstants.DT;

        double voltage_output = lowerArmFeedForward(lowerSetpoint) + kP_term + kI_term + kD_term;
        double voltage = RobotController.getBatteryVoltage();

        lowOutput = voltage_output / voltage;
        if (lowOutput > 1) {
            lowOutput = 1;
        } else if (lowOutput < -1) {
            lowOutput = -1;
        }

        LOWER_MOTOR.set(ControlMode.PercentOutput, lowOutput);

        lowPrevError = lowError;
    }

    public void pidHighPosition(){

        if (upperSetpoint > Constants.ArmConstants.MAX_ARM_ANGLE) {
            upperSetpoint = Constants.ArmConstants.MAX_ARM_ANGLE;
        }

        if (upperSetpoint < 0) {
            upperSetpoint = 0;
        }
        
        upperCurrent = getLowAngle();
        upperError = upperSetpoint - upperCurrent;
        upperAccumulator += upperError * Constants.MechanismConstants.DT;
        if (upperAccumulator > Constants.ArmConstants.ARM_SATURATION_LIMIT) {
            upperAccumulator = Constants.ArmConstants.ARM_SATURATION_LIMIT;
        } else if (upperAccumulator < -Constants.ArmConstants.ARM_SATURATION_LIMIT) {
            upperAccumulator = -Constants.ArmConstants.ARM_SATURATION_LIMIT;
        }
        double kP_term = Constants.ArmConstants.ARM_PID.kP * upperError;
        double kI_term = Constants.ArmConstants.ARM_PID.kI * upperAccumulator;
        double kD_term = Constants.ArmConstants.ARM_PID.kD * (upperError - upperPrevError) / Constants.MechanismConstants.DT;

        double voltage_output = upperArmFeedForward(upperSetpoint) + kP_term + kI_term + kD_term;
        double voltage = RobotController.getBatteryVoltage();

        upperOutput = voltage_output / voltage;
        if (upperOutput > 1) {
            // Log.info("ARM",
            //         "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this");
            upperOutput = 1;
        } else if (upperOutput < -1) {
            // Log.info("ARM",
            //         "WARNING: Tried to set power above available voltage! Saturation limit SHOULD take care of this ");
            upperOutput = -1;
        }

        /*if (Math.abs(error) < Constants.ArmConstants.ANGLE_THRESHOLD) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }
        */


        UPPER_MOTOR.set(ControlMode.PercentOutput, upperOutput);

        upperPrevError = upperError;
    }
}