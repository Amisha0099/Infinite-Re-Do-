package org.team3128.bluepercents.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team3128.common.generics.RobotConstants;

import com.kauailabs.navx.frc.AHRS;


import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.common.utility.test_suite.ErrorCatcherUtility;
import org.team3128.bluepercents.commands.*;
import org.team3128.bluepercents.autonomous.AutoPriority;
import org.team3128.bluepercents.subsystems.*;
import org.team3128.bluepercents.subsystems.Constants;
import org.team3128.bluepercents.subsystems.Arm.ArmState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.concurrent.*;

import org.team3128.common.generics.ThreadScheduler;

public class MainDrivetrain extends NarwhalRobot {

    private DriveCommandRunning driveCmdRunning;
    public Command triggerCommand;

    static FalconDrive drive = FalconDrive.getInstance();
    static Arm arm = Arm.getInstance();
    static Climber climber = new Climber();
    static BoxIntake boxIntake = BoxIntake.getInstance();
    static BellIntake bellIntake = BellIntake.getInstance();


    ExecutorService executor = Executors.newFixedThreadPool(6);
    ThreadScheduler scheduler = new ThreadScheduler();
    Thread auto;

    public Joystick joystickRight, joystickLeft;
    public ListenerManager listenerLeft, listenerRight;
    public AHRS ahrs;
    public static PowerDistributionPanel pdp;

    public NetworkTable table;
    public NetworkTable limelightTable;

    public double startTime = 0;

    public String trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";

    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();
    public Trajectory trajectory;

    public Limelight limelight;
   

    @Override
    protected void constructHardware() {
        scheduler.schedule(drive, executor);
        scheduler.schedule(boxIntake, executor);
        scheduler.schedule(bellIntake, executor);
        scheduler.schedule(arm, executor);

        driveCmdRunning = new DriveCommandRunning();

        ahrs = drive.ahrs;

        joystickRight = new Joystick(1);
        listenerRight = new ListenerManager(joystickRight);
        addListenerManager(listenerRight);

        joystickLeft = new Joystick(0);
        listenerLeft = new ListenerManager(joystickLeft);
        addListenerManager(listenerLeft);

        // initialization of limelight
        limelight = new Limelight("limelight", 26.0, 0, 0, 15);
  
        pdp = new PowerDistributionPanel(0);
        drive.resetGyro();
    }

    @Override
    protected void constructAutoPrograms() {
    }

    @Override
    protected void setupListeners() {
        listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "AlignBell");
        
        listenerRight.nameControl(new Button(3), "BoxIntakeState");
        listenerRight.nameControl(new Button(4), "BoxOuttakeState");
        listenerRight.nameControl(new Button(5), "BellIntakeState");
        listenerRight.nameControl(new POV(0), "IntakePOV");


        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "Climb");
        listenerLeft.nameControl(new Button(3), "ReleaseClimber");
        listenerLeft.nameControl(new Button(4), "ReleaseClimber");
        listenerLeft.nameControl(new Button(5), "Skewering");
        listenerLeft.nameControl(new Button(11), "EmergencyReset");
        listenerLeft.nameControl(new Button(12), "EmergencyReset");

        listenerRight.addMultiListener(() -> {
            if (driveCmdRunning.isRunning) {
                double horiz = -0.5 * listenerRight.getAxis("MoveTurn"); //0.7
                double vert = -1.0 * listenerRight.getAxis("MoveForwards");
                double throttle = -1.0 * listenerRight.getAxis("Throttle");

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");

        listenerRight.addButtonDownListener("BoxIntakeState", () -> {
            Log.info("Button3", "pressed");
            arm.setState(ArmState.INTAKEBOX);
            
        });

        listenerRight.addButtonDownListener("BoxOuttakeState", () -> {
            Log.info("Button4", "pressed");
            arm.setState(ArmState.OUTTAKEBOX);
           
        });

        listenerRight.addButtonDownListener("BellIntakeState", () -> {
            Log.info("Button5", "pressed");
            arm.setState(ArmState.INTAKEBELL);
           
        });

        listenerRight.addButtonDownListener("AlignBell", () -> {
            triggerCommand = new CmdAlignBell(drive, arm, bellIntake, ahrs, limelight, driveCmdRunning, Constants.VisionConstants.TX_OFFSET);
            triggerCommand.start();
        });

        listenerRight.addButtonUpListener("AlignBell", () -> {
            triggerCommand.cancel();
            triggerCommand = null;
        });

        listenerRight.addListener("IntakePOV", (POVValue pov) -> {
            switch (pov.getDirectionValue()) {
                case 8:
                case 2:
                case 1:
                    //intake box
                    if(arm.ARM_STATE == ArmState.INTAKEBOX){
                        boxIntake.intakingBox();
                    } 
                    
                    break;

                case 7:
                case 6:
                    //intake bell
                    if (arm.ARM_STATE == ArmState.INTAKEBELL){
                        bellIntake.intakingBell();
                    } 
                    break;
                
                case 3:
                case 4: 
                case 5: 
                    //outtake box
                    if (arm.ARM_STATE == ArmState.OUTTAKEBOX){
                        boxIntake.ejectingBox();
                    }

                    break;

                case 0:
                    
                    break;

                default:
                    break;
            }
        });

        listenerLeft.addButtonDownListener("Climb", () -> {
            Log.info("Trigger", "pressed");
            climber.setClimbingPower(Constants.ClimberConstants.CLIMB_POWER);
        });

        listenerLeft.addButtonUpListener("Climb", () -> {
            Log.info("Trigger", "released");
            climber.setClimbingPower(0.0);
        });

        listenerLeft.addButtonDownListener("ReleaseClimber", () -> {
            Log.info("Button3/4", "pressed");
            climber.setAngle(-30);
           
        });

        listenerLeft.addButtonDownListener("Skewering", () -> {
            Log.info("Button5", "pressed");
            arm.setState(ArmState.SKEWERING);
        });

        listenerLeft.addButtonDownListener("EmergencyReset", () -> {
            Log.info("MainCompBot", "EMERGENCY RESET PRESSED");
            //climber.setAngle(0);
            climber.setClimbingPower(0);
        });

        
    }

    @Override
    protected void teleopPeriodic() {
        scheduler.resume();
    }

    double maxLeftSpeed = 0;
    double maxRightSpeed = 0;
    double maxSpeed = 0;
    double minLeftSpeed = 0;
    double minRightSpeed = 0;
    double minSpeed = 0;

    double currentLeftSpeed;
    double currentRightSpeed;
    double currentSpeed;
    double currentDistance;
    

    @Override
    protected void updateDashboard() {
       /*
        SmartDashboard.putString("hopper update count", String.valueOf(//hopper.hopper_update_count));
        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("ball_count", 0);

        Log.info("HOPPER", "" + hopper.SENSOR_1_STATE);

        if (arm.getLimitStatus()) {
            arm.ARM_MOTOR_LEADER.setSelectedSensorPosition(0);
            arm.ARM_MOTOR_FOLLOWER.setSelectedSensorPosition(0);
        }

        currentLeftSpeed = drive.getLeftSpeed();
        currentRightSpeed = drive.getRightSpeed();

        currentSpeed = drive.getSpeed();
    

        SmartDashboard.putNumber("Left Velocity", currentLeftSpeed);
        SmartDashboard.putNumber("Right Velocity", currentRightSpeed);
*/
    }

    @Override
    protected void teleopInit() {
        scheduler.resume();
        limelight.setLEDMode(LEDMode.OFF);
        arm.LOWER_MOTOR.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        arm.UPPER_MOTOR.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        driveCmdRunning.isRunning = true;
    }

    @Override
    protected void autonomousInit() {
        scheduler.resume();
        drive.resetGyro();
        //add auto stuff
        Command auto = new AutoPriority(drive, arm, bellIntake, ahrs, limelight, driveCmdRunning, 10000, scheduler);
        auto.start();
    }

    @Override
    protected void disabledInit() {
        limelight.setLEDMode(LEDMode.OFF);
        arm.LOWER_MOTOR.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        arm.UPPER_MOTOR.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainDrivetrain::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}