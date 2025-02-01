// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeUtil.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopDriveCommand extends Command {
    private final SwerveSubsystem  swerve;
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   vZ;

    private final double maxSwerveVelocity;
    private final double maxSwerveAngularVelocity;

    double autoCenterKp = 0.028;
    double autoTurnKp = 0.02;

    private final Limelight limeLightReceiver;
    private final BooleanSupplier isTurningToAngle;
    private final DoubleSupplier targetAngle;

    private final BooleanSupplier isTurningToSeenAprilTagAngle;
    private boolean aprilTagAngleIsKnown = false;
    private double aprilTagAngle;

    public TeleopDriveCommand(
        SwerveSubsystem swerve, 
        DoubleSupplier vX, 
        DoubleSupplier vY, 
        DoubleSupplier vZ,
        Limelight lr, 
        BooleanSupplier isTurningToAngle,
        DoubleSupplier targetAngle,
        BooleanSupplier isTurningToSeenAprilTagAngle
    ) {
            this.swerve = swerve;
            this.limeLightReceiver = lr;
            this.vX = vX;
            this.vY = vY;
            this.vZ = vZ;
            this.isTurningToAngle = isTurningToAngle;
            this.targetAngle = targetAngle;
            this.isTurningToSeenAprilTagAngle = isTurningToSeenAprilTagAngle;

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
        }

        if(isTurningToSeenAprilTagAngle.getAsBoolean()) {

            // if angle is known
            if(this.aprilTagAngleIsKnown) {
                angVelocity = autoTurnKp * Math.IEEEremainder(aprilTagAngle - swerve.getHeading().getDegrees(), 360); 

            // if angle is not known
            } else {
                // if april tag is seen
                
                    // log the april tag id

                    // get the angle of the april tag from the field library

                    this.aprilTagAngleIsKnown = true;
                
                // if april tag is not seen

                angVelocity = getAngularVelocityFromStick();

            }
                
        } else if(isTurningToAngle.getAsBoolean()) {
            angVelocity = autoTurnKp * Math.IEEEremainder(targetAngle.getAsDouble() - swerve.getHeading().getDegrees(), 360);
        
        } else {
            angVelocity = getAngularVelocityFromStick();
        }



        double xVelocity   = Math.pow(vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(vY.getAsDouble(), 3);

        if(swerve.isRedAlliance()) {
            xVelocity = xVelocity * -1;
            yVelocity = yVelocity * -1;
        }
        
        swerve.driveFieldOriented(
            new ChassisSpeeds(
                xVelocity * maxSwerveVelocity * 0.8,
                yVelocity * maxSwerveVelocity * 0.8,
                angVelocity * maxSwerveAngularVelocity * .75
            )
        );
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
