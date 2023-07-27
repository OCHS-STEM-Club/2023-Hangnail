// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.PipedInputStream;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase {

  private DriveSubsystem swerve;
  private boolean isRelative;
  private double goal;
  private HolonomicDriveController holonomicDriveController;
  private Pose2d startPos = new Pose2d();
  private Pose2d targPose2d = new Pose2d();

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveSubsystem swerve, double angle, boolean isRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.swerve = swerve;
    this.goal = angle;
    this.isRelative = isRelative;

    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0001, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    holonomicDriveController = new HolonomicDriveController(xController, yController, thetaController);
    holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(5)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = swerve.getPose();
    if (isRelative) {
      targPose2d = new Pose2d(startPos.getTranslation(),
        startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
    } else {
      targPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose2d = swerve.getPose();
    ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d, 
      targPose2d, 0, targPose2d.getRotation());
    SwerveModuleState[] swerveModuleStates =
      Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(swerveModuleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  SwerveModuleState[] swerveModuleStates =
    Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  swerve.setModuleStates(swerveModuleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return holonomicDriveController.atReference();
  }
}
