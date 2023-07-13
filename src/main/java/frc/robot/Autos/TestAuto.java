// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Autos.AutoBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class TestAuto extends AutoBase {
    public TestAuto(DriveSubsystem swerve) {
        super(swerve);
        PathPlannerTrajectory testPath = PathPlanner.loadPath("Test Path", 2, 1);
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(testPath);
        PathPlannerState initialState = testPath.getInitialState();
        // TurnToAngle firstCommand = new TurnToAngle(swerve, 250, false);

        addCommands(new InstantCommand(() -> swerve.zeroHeading()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            firstCommand);

    }
    
}