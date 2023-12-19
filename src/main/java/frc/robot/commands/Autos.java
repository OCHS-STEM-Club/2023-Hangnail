// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServerThread;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class Autos {
     public static CommandBase exampleAuto(DriveSubsystem swerve) {

      //  public PathPlannerTrajectory (String pathName) {
            // PathConstraints constraints = PathPlanner.getConstraintsFromPath(pathName);
            // PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath(pathName, constraints, false);
            // return ppTrajectory;
            // }
//         List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup("New New Path", new PathConstraints(4, 3));
//         // This is just an example event map. It would be better to have a constant, global event map
//         // in your code that will be used by all path following commands.
//         HashMap<String, Command> eventMap = new HashMap<>();
//         eventMap.put("marker1", new PrintCommand("Passed marker 1"));
  
//         // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
//         // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
//         return Commands.sequence(example1));
//       }
//   //    swerve.postTrajectory(example);
//       return Commands.sequence(new FollowTrajectory(swerve, example, true));
   
    return null;
    }
}

