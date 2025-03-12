// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveToPose extends Command {
    private final SwerveSubsystem swerve;
    Supplier<Pose2d> targetPose;
    double targetRotation;
    private final double maxSwerveVelocity;
    private double driveKp = 4.0;
    private double turnKp = 0.045;
    private PIDController xController;
    private PIDController yController;
    private double xVelocity;
    private double yVelocity;
    private double angVelocity;
    private boolean targetIsSet = false; 
    private int counter = 0;

    public AutoDriveToPose(
        SwerveSubsystem swerve,
        Supplier<Pose2d> targetPose
    ) {
        this.swerve = swerve;
        this.targetPose = targetPose;

        this.maxSwerveVelocity = swerve.getSwerveDrive().getMaximumChassisVelocity();

        xController = new PIDController(driveKp, 0.0, 0.0);
        yController = new PIDController(driveKp, 0.0, 0.0);

        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xController.setTolerance(0.05, 1.0); 
        yController.setTolerance(0.05, 1.0);
        counter = 0;
        targetIsSet = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!targetIsSet) {
            // check if target pose is not null
            if(targetPose.get() != null) {
                try {
                    xController.setSetpoint(targetPose.get().getX());
                    yController.setSetpoint(targetPose.get().getY());
                    targetRotation = targetPose.get().getRotation().getDegrees();
                    targetIsSet = true;

                } catch (Exception e) {
                    // TODO: handle exception
                }
                
            }

            swerve.driveFieldOriented(
                new ChassisSpeeds(0.0, 0.0, 0.0)
            );
        } else {
            counter++;

            xVelocity = xController.calculate(swerve.getPose().getX());
            yVelocity = yController.calculate(swerve.getPose().getY());
            angVelocity = turnKp * Math.IEEEremainder(targetRotation - swerve.getHeading().getDegrees(), 360);



            swerve.driveFieldOriented(
                new ChassisSpeeds(
                    xVelocity * maxSwerveVelocity * 0.2,
                    yVelocity * maxSwerveVelocity * 0.2,
                    angVelocity
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return counter > 70;
    }
}
