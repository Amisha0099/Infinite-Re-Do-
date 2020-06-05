package org.team3128.bluepercents.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import com.revrobotics.CANEncoder;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;
import org.team3128.common.utility.enums.Direction;


public class Climber extends Threaded {
    public LazyCANSparkMax LEFT_MOTOR, RIGHT_MOTOR, TURNING_MOTOR;
    public CANEncoder TURNING_ENCODER;

    public double angle = 0;
    double current = 0;
    double error = 0;
    public double output = 0;
    double accumulator = 0;
    double prevError = 0;

    public Climber() {
        configMotors();
        configEncoders();
    }

    private void configMotors() {
       LEFT_MOTOR = new LazyCANSparkMax(Constants.ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
       RIGHT_MOTOR = new LazyCANSparkMax(Constants.ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
       TURNING_MOTOR = new LazyCANSparkMax(Constants.ClimberConstants.TURNING_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    }

    public void configEncoders(){
        TURNING_ENCODER = TURNING_MOTOR.getEncoder();
    }

    public void setClimbingPower(double climbingPower){
        LEFT_MOTOR.set(-climbingPower);
        RIGHT_MOTOR.set(-climbingPower);
    }
    
    @Override
    public void update(){
        current = getAngle();
        error = angle - current;
        accumulator += error * Constants.MechanismConstants.DT;
        if (accumulator > Constants.ClimberConstants.CLIMBER_SATURATION_LIMIT) {
            accumulator = Constants.ClimberConstants.CLIMBER_SATURATION_LIMIT;
        } else if (accumulator < -Constants.ClimberConstants.CLIMBER_SATURATION_LIMIT) {
            accumulator = -Constants.ClimberConstants.CLIMBER_SATURATION_LIMIT;
        }

        double kP_term = Constants.ClimberConstants.CLIMBER_PID.kP * error;
        double kI_term = Constants.ClimberConstants.CLIMBER_PID.kI * accumulator;
        double kD_term = Constants.ClimberConstants.CLIMBER_PID.kD * (error - prevError) / Constants.MechanismConstants.DT;

        double voltage_output = climberFeedForward(angle) + kP_term + kI_term + kD_term;
        double voltage = RobotController.getBatteryVoltage();

        output = voltage_output / voltage;

        if (output > 1) {
            output = 1;
        } else if (output < -1) {
            output = -1;
        }

        TURNING_MOTOR.set(output);
        
        prevError = error;
    }

    public double climberFeedForward(double desired) {
        return 0.3; 
    }


    public void setAngle(double desiredAngle){
        angle = desiredAngle;
    }

    public double getEncoderPos() {
        return TURNING_ENCODER.getPosition();
    }

    public double getAngle() {
        return (((getEncoderPos() / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)) * 360) % 360; 
    }

}