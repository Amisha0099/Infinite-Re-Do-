package org.team3128.bluepercents.commands;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Angle;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

import org.team3128.bluepercents.subsystems.Constants;
import org.team3128.bluepercents.subsystems.Arm.ArmState;
import org.team3128.bluepercents.subsystems.Constants.BellIntakeConstants;
import org.team3128.bluepercents.subsystems.BellIntake;
import org.team3128.bluepercents.subsystems.*;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.bluepercents.commands.*;

public class CmdTurnWheel extends Command{
    FalconDrive drive;
    Arm arm;
    BellIntake bellIntake;

    DriveCommandRunning cmdRunning;
    AHRS ahrs;
    
    public CmdTurnWheel(FalconDrive drive, Arm arm, BellIntake bellIntake, AHRS ahrs, DriveCommandRunning cmdRunning){
        this.drive = drive;
        this.arm = arm;
        this.bellIntake = bellIntake;
        this.ahrs = ahrs;
        this.cmdRunning = cmdRunning;
    }

    @Override
    protected void initialize(){
        cmdRunning.isRunning = false;
    }

    @Override
    protected void execute(){
        arm.setState(ArmState.SPINWHEEL);
        
    }

    @Override
    protected boolean isFinished(){
        return true;
    }

    @Override
    protected void end(){
        drive.stopMovement();
        cmdRunning.isRunning = true;
    }

    @Override
    protected void interrupted(){
        end();
    }

}