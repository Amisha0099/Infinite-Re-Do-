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

import org.team3128.bluepercents.subsystems.Constants.BellIntakeConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

public class BellIntake extends Threaded {

    public enum BellState {
        INTAKING, EJECTING;

        private BellState() {

        }
    }

    public LazyCANSparkMax BELL_MOTOR;
    public DigitalInput BELL_SENSOR;
    public CANEncoder BELL_ENCODER;
    public BellState bellState;

    private static final BellIntake instance = new BellIntake();

    private boolean detectBool = false;
    public boolean SENSOR_BELL_STATE = false;
    public double bellInitialPos = 0;

    public boolean intakeBellButton = false;
    public boolean ejectBellButton = false;

    public boolean intakingBell = false;
    public boolean ejectingBell = false;

    public static BellIntake getInstance() {
        return instance;
    }

    private BellIntake() {
        configMotors();
        configSensors();
        configEncoders();
    }

    public void update(){
        SENSOR_BELL_STATE = detectsBell();
        
        //intake the bell
        if(intakeBellButton = true){
            intakingBell = true;
            intakeBellButton = false;
        }

        if((intakingBell = true) && (!SENSOR_BELL_STATE)){
            BELL_MOTOR.set(BellIntakeConstants.BELL_MOTOR_ON_VALUE);
        }
        else{
            BELL_MOTOR.set(BellIntakeConstants.BELL_MOTOR_OFF_VALUE);
            
            intakingBell = false;
        }

        //eject the bell
        if(ejectBellButton = true){
            ejectingBell = true;
            bellInitialPos = BELL_ENCODER.getPosition();

            ejectBellButton = false;
        }

        if(ejectingBell){
            if(Math.abs(BELL_ENCODER.getPosition()) < Math.abs(bellInitialPos + Constants.BellIntakeConstants.BELL_OFFSET_VALUE)){
                BELL_MOTOR.set(-BellIntakeConstants.BELL_MOTOR_ON_VALUE);
            }
            else{
                BELL_MOTOR.set(BellIntakeConstants.BELL_MOTOR_OFF_VALUE);

                ejectingBell = false;
                isEmpty(true);
            }
        }
        
    }

    private void configMotors(){
        BELL_MOTOR = new LazyCANSparkMax(Constants.BellIntakeConstants.BELL_MOTOR_ID, MotorType.kBrushless);
    }

    private void configSensors() {
        BELL_SENSOR = new DigitalInput(Constants.BellIntakeConstants.SENSOR_BELL_ID);
    }

    private void configEncoders(){
        BELL_ENCODER = BELL_MOTOR.getEncoder();
    }

    public boolean detectsBell(){
        if(BELL_SENSOR.get()){
            detectBool = true;
        }
        else {
            detectBool = false;
        }
        return detectBool;
    }

    public void ejectingBell(){
        ejectBellButton = true;
    }

    public void intakingBell(){
        intakeBellButton = true;
    }
    
    public boolean isEmpty(boolean isEmpty){
        return isEmpty;
    }
}