package org.team3128.bluepercents.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

//import org.team3128.bluepercents.autonomous.CmdArmInitialize;
import org.team3128.bluepercents.autonomous.CmdDrive;
import org.team3128.bluepercents.autonomous.CmdAutoTrajectory;

import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.drive.Drive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.bluepercents.autonomous.*;
import org.team3128.bluepercents.commands.*;
import org.team3128.bluepercents.subsystems.Constants;
//import org.team3128.bluepercents.subsystems.Arm.ArmState;
import org.team3128.bluepercents.subsystems.*;
import org.team3128.common.utility.Log;
import com.kauailabs.navx.frc.AHRS;
import org.team3128.common.generics.ThreadScheduler;

public class AutoPriority extends CommandGroup{
    public AutoPriority(FalconDrive drive, AHRS ahrs, Limelight limelight, DriveCommandRunning cmdRunning, double timeoutMs, ThreadScheduler scheduler){
        addSequential(new CmdDrive(drive, 1));
        addSequential(new CmdAutoTrajectory(drive, 130, 0.5, 10000, 
            new Pose2D(0, 0, Rotation2D.fromDegrees(0)),
            new Pose2D(-51.771250* Constants.MechanismConstants.inchesToMeters, 0 * Constants.MechanismConstants.inchesToMeters, Rotation2D.fromDegrees(0)))); //move from initiation line back to smartbell goal
        //drop off preloaded smartbell
        addSequential(new CmdAutoTrajectory(drive, 130, 0.5, 10000, 
            new Pose2D(0, 0, Rotation2D.fromDegrees(0)),
            new Pose2D(276.476124* Constants.MechanismConstants.inchesToMeters, 0 * Constants.MechanismConstants.inchesToMeters, Rotation2D.fromDegrees(0)))); //move from smartbell goal to smartbell depot
        //pick up smarbell from depot
        addSequential(new CmdAutoTrajectory(drive, 130, 0.5, 10000, 
            new Pose2D(0, 0, Rotation2D.fromDegrees(0)),
            new Pose2D(-276.476124* Constants.MechanismConstants.inchesToMeters, 0 * Constants.MechanismConstants.inchesToMeters, Rotation2D.fromDegrees(0)))); //move from smartbell depot to smartbell goal
    }

}