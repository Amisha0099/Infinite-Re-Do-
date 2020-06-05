package org.team3128.bluepercents.commands;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Pipeline;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.limelight.StreamMode;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Angle;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team3128.bluepercents.subsystems.Constants;
import org.team3128.bluepercents.subsystems.Arm.ArmState;
import org.team3128.bluepercents.subsystems.Constants.BellIntakeConstants;
import org.team3128.bluepercents.subsystems.BellIntake;
import org.team3128.bluepercents.subsystems.*;

import com.kauailabs.navx.frc.AHRS;


import org.team3128.bluepercents.commands.*;

public class CmdAlignBell extends Command {
    FalconDrive drive;
    Arm arm;
    BellIntake bellIntake;

    AHRS ahrs;

    Limelight limelight;

    double decelerationStartDistance, decelerationEndDistance;
    DriveCommandRunning cmdRunning;

    private PIDConstants visionPID;

    private double goalHorizontalOffset;

    private double currentHorizontalOffset;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower;

    private double leftPower, rightPower;
    
    int targetFoundCount;
    int thresholdCount = 0;

    private enum HorizontalOffsetFeedbackDriveState {
        SEARCHING, FEEDBACK;
    }

    private HorizontalOffsetFeedbackDriveState aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

    public CmdAlignBell(FalconDrive drive, Arm arm, BellIntake bellIntake, AHRS ahrs, Limelight limelight,
            DriveCommandRunning cmdRunning, double goalHorizontalOffset) {
        this.drive = drive;
        this.arm = arm;
        this.bellIntake = bellIntake;
        this.ahrs = ahrs;
        this.limelight = limelight;
        this.visionPID = Constants.VisionConstants.VISION_PID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

    }

    @Override
    protected void initialize() {
        limelight.setLEDMode(LEDMode.ON);
        cmdRunning.isRunning = false;
    }

    @Override
    protected void execute() {
        switch (aimState) {
            case SEARCHING:
                if (limelight.hasValidTarget()) {
                    targetFoundCount += 1;
                } else {
                    targetFoundCount = 0;
                }

                if (targetFoundCount > 5) {

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    previousTime = RobotController.getFPGATime();
                    previousError = goalHorizontalOffset - currentHorizontalOffset;

                    cmdRunning.isRunning = true;

                    aimState = HorizontalOffsetFeedbackDriveState.FEEDBACK;
                }

                break;

            case FEEDBACK:
                cmdRunning.isRunning = false;
                if (!limelight.hasValidTarget()) {
                    aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

                } else {

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime();
                    currentError = goalHorizontalOffset - currentHorizontalOffset;

                    /**
                     * PID feedback loop for the left and right powers based on the horizontal
                     * offset errors.
                     */
                    feedbackPower = 0;

                    feedbackPower += visionPID.kP * currentError;
                    feedbackPower += visionPID.kD * (currentError - previousError) / (currentTime - previousTime);

                    leftPower = RobotMath.clamp(-feedbackPower, -1, 1);
                    rightPower = RobotMath.clamp(feedbackPower, -1, 1);

                    drive.setWheelPower(new DriveSignal(leftPower, rightPower));

                    previousTime = currentTime;
                    previousError = currentError;
                }

                break;

        }

    }

    @Override
    protected boolean isFinished() {
        if ((Math.abs(currentError) < Constants.VisionConstants.TX_THRESHOLD)) {
            thresholdCount++;
            if(thresholdCount == 5){
                bellIntake.ejectingBell();
                thresholdCount = 0;
                return true;
            }
            return false;
            
        } 
        else{
            thresholdCount = 0;
            return false;
        }
    }

    @Override
    protected void end() {
        limelight.setLEDMode(LEDMode.OFF);
        drive.stopMovement();
        cmdRunning.isRunning = true;
    }

    @Override
    protected void interrupted() {
        end();
    }
}