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

    public enum ActionState {
        STANDBY, INTAKING, EJECTING;

        private ActionState() {

        }
    }

    public LazyCANSparkMax LEFT_BOX_MOTOR;
    public LazyCANSparkMax RIGHT_BOX_MOTOR;
    public DigitalInput BOX_SENSOR;

    private static final BoxIntake instance = new BoxIntake();

    public ActionState actionState;
    public boolean SENSOR_BOX_STATE = false;


    public static BoxIntake getInstance() {
        return instance;
    }

    private BoxIntake() {
        configMotors();
        configSensors();
    }

    public void update(){

    }

    private void configMotors(){
        LEFT_BOX_MOTOR = new LazyCANSparkMax(Constants.BoxIntakeConstants.LEFT_BOX_MOTOR_ID, MotorType.kBrushless);
        RIGHT_BOX_MOTOR = new LazyCANSparkMax(Constants.BoxIntakeConstants.RIGHT_BOX_MOTOR_ID, MotorType.kBrushless);
    }

    private void configSensors() {
        BOX_SENSOR = new DigitalInput(Constants.BoxIntakeConstants.SENSOR_BOX_ID);
    }

    public void setAction(ActionState state){
        this.actionState = state;
        if(state == ActionState.INTAKING){
            LEFT_BOX_MOTOR.set(BoxIntakeConstants.BOX_MOTOR_ON_VALUE);
            RIGHT_BOX_MOTOR.set(-BoxIntakeConstants.BOX_MOTOR_ON_VALUE);

        }

        if(state == ActionState.EJECTING){
            LEFT_BOX_MOTOR.set(-BoxIntakeConstants.BOX_MOTOR_ON_VALUE);
            RIGHT_BOX_MOTOR.set(BoxIntakeConstants.BOX_MOTOR_ON_VALUE);

        }

        if(state == ActionState.STANDBY){
            LEFT_BOX_MOTOR.set(0);
            RIGHT_BOX_MOTOR.set(0);

        }
    }
}