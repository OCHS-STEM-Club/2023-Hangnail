// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos.Pathplanner;

/** Add your docs here. */
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Autos.AutoBase;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class StraightMobility extends AutoBase {
    public StraightMobility(DriveSubsystem swerve) {
        super(swerve);
        //List<PathPlannerTrajectory> testPath1 = PathPlanner.loadPathGroup("Segment 2 Test", 2, 1);
        PathPlannerTrajectory testPath2 = PathPlanner.loadPath("Segment 2 Test", 2, 1);

       // return Commands.sequence(AutoBase.);

        //PPSwerveControllerCommand firstCommand1 = baseSwerveCommand(testPath1);
        PPSwerveControllerCommand secondCommand = baseSwerveCommand(testPath2);

        PathPlannerState initialState = testPath2.getInitialState();

        //  SequentialCommandGroup driveSegment1 =
        //      new SequentialCommandGroup(testPath2);

        //SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(null, null, null, null, null, null, null)
        
        //   SequentialCommandGroup driveSegment2 =
        //         new TurnToAngle(swerve, -90, false).andThen(secondCommand);


        addCommands(new InstantCommand(() -> swerve.zeroHeading()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
           secondCommand);
            //        new SequentialCommandGroup(turnSegment1));

    }
    
}