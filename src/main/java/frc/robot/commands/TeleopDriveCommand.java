// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeUtil.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDriveCommand extends Command {
    private final SwerveSubsystem  swerve;
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   vZ;
    private final DoubleSupplier liftPosition;

    private final double maxSwerveVelocity;
    private final double maxSwerveAngularVelocity;

    double autoCenterKp = 0.028;
    double autoTurnKp = 0.045;

    private final Limelight limelight;
    private final BooleanSupplier isTurningToAngle;
    private final DoubleSupplier targetAngle;

    private final BooleanSupplier isTurningToSeenAprilTagAngle;
    private final BooleanSupplier isFieldOriented;
    private boolean aprilTagAngleIsKnown = false;
    private double aprilTagAngle;
    private int seenAprilTag;
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    private double speedModifier = 0.7;

    public TeleopDriveCommand(
        SwerveSubsystem swerve, 
        DoubleSupplier vX, 
        DoubleSupplier vY, 
        DoubleSupplier vZ,
        BooleanSupplier isTurningToAngle,
        DoubleSupplier targetAngle,
        BooleanSupplier isTurningToSeenAprilTagAngle,
        BooleanSupplier isFieldOriented,
        DoubleSupplier liftPosition
    ) {
            this.swerve = swerve;
            this.limelight = swerve.limelight3;
            this.vX = vX;
            this.vY = vY;
            this.vZ = vZ;
            this.isTurningToAngle = isTurningToAngle;
            this.targetAngle = targetAngle;
            this.isTurningToSeenAprilTagAngle = isTurningToSeenAprilTagAngle;
            this.isFieldOriented = isFieldOriented;
            this.liftPosition = liftPosition;

            this.maxSwerveVelocity = swerve.getSwerveDrive().getMaximumChassisVelocity();
            this.maxSwerveAngularVelocity = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();

            addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double angVelocity;
        if(!isTurningToSeenAprilTagAngle.getAsBoolean()) {
            this.aprilTagAngleIsKnown = false;
            this.seenAprilTag = 0;
        }
        
        if(isTurningToSeenAprilTagAngle.getAsBoolean()) {

            // if angle is known
            if(this.aprilTagAngleIsKnown) {
                angVelocity = autoTurnKp * Math.IEEEremainder(aprilTagAngle - swerve.getHeading().getDegrees(), 360); 

            // if angle is not known
            } else {

                // if april tag is seen
                if(limelight.tagIsSeen()) {
                    // log the april tag id
                    seenAprilTag = limelight.getPrimaryAprilTagID();

                    // get the angle of the april tag from the field library
                    Pose3d tagPose = fieldLayout.getTagPose(seenAprilTag).get();
                    aprilTagAngle = Math.toDegrees(tagPose.getRotation().getAngle());
                    

                    this.aprilTagAngleIsKnown = true;

                } 
                angVelocity = getAngularVelocityFromStick();
            }
                
        } else if(isTurningToAngle.getAsBoolean()) {
            angVelocity = autoTurnKp * Math.IEEEremainder(targetAngle.getAsDouble() - swerve.getHeading().getDegrees(), 360);
        
        } else {
            angVelocity = getAngularVelocityFromStick() * maxSwerveAngularVelocity * .5;
        }



        double xVelocity   = Math.pow(vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(vY.getAsDouble(), 3);

        if(swerve.isRedAlliance()) {
            xVelocity = xVelocity * -1;
            yVelocity = yVelocity * -1;
        }

        
        // field oriented
        if(isFieldOriented.getAsBoolean()) {
            if(liftPosition.getAsDouble() < 130.0) speedModifier = 0.9;

            swerve.driveFieldOriented(
                new ChassisSpeeds(
                    xVelocity * maxSwerveVelocity * speedModifier,
                    yVelocity * maxSwerveVelocity * speedModifier,
                    angVelocity
                )
            );

        // robot relative
        } else {
            swerve.drive(
                new ChassisSpeeds(
                    xVelocity * maxSwerveVelocity * 0.7,
                    yVelocity * maxSwerveVelocity * 0.7,
                    angVelocity 
                )
            );
        }

    }



    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public double getAngularVelocityFromStick() {
        return Math.pow(vZ.getAsDouble(), 3);
    }
}
