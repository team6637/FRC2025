// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveRobotRelative extends Command {
    private final SwerveSubsystem swerve;
    private final double maxSwerveVelocity;
    private double xKp = 0.3;
    private double yKp = 0.1;
    private double turnKp = 0.046;
    private PIDController xController;
    private PIDController yController;
    private double xVelocity;
    private double yVelocity;
    private double angVelocity;
    private int counter = 0;
    private double currentArea;
    private double targetY;
    private double targetRotation;

    public AutoDriveRobotRelative(
        SwerveSubsystem swerve) {
        this.swerve = swerve;

        this.maxSwerveVelocity = swerve.getSwerveDrive().getMaximumChassisVelocity();

        xController = new PIDController(xKp, 0.0, 0.0);
        yController = new PIDController(yKp, 0.0, 0.0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        counter = 0;
    }

    @Override
    public void execute() {
        int aprilTagId = swerve.limelightFront.getPrimaryAprilTagID();
        //System.out.println("Tag is seen: " + swerve.limelightFront.tagIsSeen());
        //System.out.println("April Tag Id: " + aprilTagId);
        if(swerve.limelightFront.tagIsSeen() && aprilTagId != 0 && aprilTagId != -1) {
     
            Pose2d aprilTagPose = swerve.fieldLayout.getTagPose(aprilTagId).get().toPose2d();

            targetRotation = aprilTagPose.getRotation().getDegrees() + 180;
            targetY = swerve.limelightFront.getX() * -1;
            currentArea = swerve.limelightFront.getA();
            
            counter++;
            
            xVelocity = xController.calculate(currentArea, 16.9);
            yVelocity = yController.calculate(0.0, targetY);
            angVelocity = turnKp * Math.IEEEremainder(targetRotation - swerve.getHeading().getDegrees(), 360);

            xVelocity = xVelocity * maxSwerveVelocity * 0.11;

            swerve.drive(
                new ChassisSpeeds(
                    xVelocity,
                    yVelocity * maxSwerveVelocity * 0.2,
                    angVelocity
                )
            );
    

        } else {
            swerve.drive(
                new ChassisSpeeds(maxSwerveVelocity * 0.15, 
                0.0, 0.0)
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }

    @Override
    public boolean isFinished() {
        return counter > 74;
    }
}
