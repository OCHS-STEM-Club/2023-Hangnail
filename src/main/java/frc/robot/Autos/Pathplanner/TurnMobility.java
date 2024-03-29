// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos.Pathplanner;

import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Autos.AutoBase;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class TurnMobility extends AutoBase {
    public TurnMobility(DriveSubsystem swerve) {
        super(swerve);
        PathPlannerTrajectory testPath = PathPlanner.loadPath("Comp Auto Turn", 2, 1);
       // PathPlannerTrajectory testPath2 = PathPlanner.loadPath("Segment 2 Test", 2, 1);

        PPSwerveControllerCommand firstCommand = baseSwerveCommand(testPath);
        //PPSwerveControllerCommand secondCommand = baseSwerveCommand(testPath2);

        PathPlannerState initialState = testPath.getInitialState();

        //   SequentialCommandGroup driveSegment1 =
        //       new SequentialCommandGroup(firstCommand);

        
        //   SequentialCommandGroup driveSegment2 =
        //         new TurnToAngle(swerve, -90, false).andThen(secondCommand);


        addCommands(new InstantCommand(() -> swerve.zeroHeading()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            firstCommand);
            //        new SequentialCommandGroup(turnSegment1));

    }
    
}