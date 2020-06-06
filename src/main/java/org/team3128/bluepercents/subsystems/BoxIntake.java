package org.team3128.bluepercents.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyVictorSPX;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.LinkedList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.bluepercents.subsystems.Constants.BoxIntakeConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

public class BoxIntake extends Threaded {

    public enum BoxState {
        STANDBY, INTAKING, EJECTING;

        private BoxState() {

        }
    }

    public LazyCANSparkMax LEFT_BOX_MOTOR;
    public LazyCANSparkMax RIGHT_BOX_MOTOR;
    public CANEncoder BOX_ENCODER;
    public DigitalInput BOX_SENSOR;

    public boolean intakeBoxButton = false;
    public boolean ejectBoxButton = false; 

    public boolean intakingBox = false;
    public boolean ejectingBox = false;

    private static final BoxIntake instance = new BoxIntake();
    public BoxState BoxState;

    public boolean SENSOR_BOX_STATE = false;
    private boolean detectBool;
    public double boxInitialPos = 0;

    public static BoxIntake getInstance() {
        return instance;
    }

    private BoxIntake() {
        configMotors();
        configSensors();
        configEncoders();
    }

    public void update(){
        SENSOR_BOX_STATE = detectsBox();
    
        //intake the box
        if(intakeBoxButton = true){
            intakingBox = true;
            intakeBoxButton = false;
        }

        if((intakingBox = true) && (!SENSOR_BOX_STATE)){
            LEFT_BOX_MOTOR.set(BoxIntakeConstants.BOX_MOTOR_ON_VALUE);
            RIGHT_BOX_MOTOR.set(-BoxIntakeConstants.BOX_MOTOR_ON_VALUE);

        }
        else{
            LEFT_BOX_MOTOR.set(BoxIntakeConstants.BOX_MOTOR_OFF_VALUE);
            RIGHT_BOX_MOTOR.set(-BoxIntakeConstants.BOX_MOTOR_OFF_VALUE);
            
            intakingBox = false;
        }
        
        //eject the box
        if(ejectBoxButton = true){
            ejectingBox = true;
            boxInitialPos = BOX_ENCODER.getPosition();

            ejectBoxButton = false;
        }

        if(ejectingBox){
            if(Math.abs(BOX_ENCODER.getPosition()) < Math.abs(boxInitialPos + 
                        Constants.BoxIntakeConstants.BOX_OFFSET_VALUE)){
                LEFT_BOX_MOTOR.set(-BoxIntakeConstants.BOX_MOTOR_ON_VALUE);
                RIGHT_BOX_MOTOR.set(BoxIntakeConstants.BOX_MOTOR_ON_VALUE);
            }
            else{
                LEFT_BOX_MOTOR.set(BoxIntakeConstants.BOX_MOTOR_OFF_VALUE);
                RIGHT_BOX_MOTOR.set(BoxIntakeConstants.BOX_MOTOR_OFF_VALUE);

                ejectingBox = false;
            }
        }
        
    }

    private void configMotors(){
        LEFT_BOX_MOTOR = new LazyCANSparkMax(Constants.BoxIntakeConstants.LEFT_BOX_MOTOR_ID, MotorType.kBrushless);
        RIGHT_BOX_MOTOR = new LazyCANSparkMax(Constants.BoxIntakeConstants.RIGHT_BOX_MOTOR_ID, MotorType.kBrushless);
    }

    private void configSensors() {
        BOX_SENSOR = new DigitalInput(Constants.BoxIntakeConstants.SENSOR_BOX_ID);
    }

    private void configEncoders(){
        BOX_ENCODER = LEFT_BOX_MOTOR.getEncoder();
    }

    public boolean detectsBox(){
        if(BOX_SENSOR.get()){
            detectBool = true;
        }
        else {
            detectBool = false;
        }
        return detectBool;
    }

    public void intakingBox(){
        intakeBoxButton = true;
    }

    public void ejectingBox(){
        ejectBoxButton = true;
    }
}