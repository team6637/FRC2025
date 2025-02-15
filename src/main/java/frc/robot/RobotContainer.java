// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of You aren't reading this eric?
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimeUtil.Limelight;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    Joystick driverJoystick = new Joystick(0);
    Joystick operatorJoystick = new Joystick(1);
    private final Limelight limelightReceiver = new Limelight("limelight3");
    private final Lift lift = new Lift();
    private final Arm arm = new Arm();
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final CoralIntake coralIntake = new CoralIntake();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    private boolean isTurningToAngle = false;
    private double driveTargetAngle = 0.0;
    private boolean isTurningToSeenAprilTagAngle = false;

    public RobotContainer() {
        drivebase.setDefaultCommand(
            new TeleopDriveCommand(
                drivebase, 
                ()-> -MathUtil.applyDeadband(driverJoystick.getY(), OperatorConstants.Y_DEADBAND),
                ()-> -MathUtil.applyDeadband(driverJoystick.getX(), OperatorConstants.Y_DEADBAND),
                ()-> -MathUtil.applyDeadband(driverJoystick.getTwist(), OperatorConstants.TWIST_DEADBAND),
                limelightReceiver,
                ()->isTurningToAngle,
                ()->driveTargetAngle,
                ()->isTurningToSeenAprilTagAngle
            )
        ); 

        configureBindings();


    }

    private void configureBindings() {
        // DRIVER BUTTONS

        // Intake In and Algae In (3)
        new JoystickButton(driverJoystick, 3).onTrue(
            new InstantCommand(()->coralIntake.collect(), coralIntake)
        ).onFalse(
            new InstantCommand(()->coralIntake.stop(), coralIntake)
        );
        // Intake Out and Algae Out (1)
        new JoystickButton(driverJoystick, 1).onTrue(
            new InstantCommand(()->coralIntake.score(), algaeIntake)
        ).onFalse(
            new InstantCommand(()->coralIntake.stop(), algaeIntake)
        );

        // Algae Down (11)
        new JoystickButton(driverJoystick, 11).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()->algaeIntake.down(), algaeIntake),
                new WaitUntilCommand(()->algaeIntake.atSetpoint()),
                new InstantCommand(()->algaeIntake.setUsingPID(false), algaeIntake),
                new InstantCommand(()->algaeIntake.stop(), algaeIntake)

            )
        );

        // Algae Up (16)
        new JoystickButton(driverJoystick, 16).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()->algaeIntake.setUsingPID(true), algaeIntake),
                new InstantCommand(()->algaeIntake.up(), algaeIntake)
            )
        );

        // Lift Up POV 0
        new POVButton(driverJoystick, 0).onTrue(
            new RunCommand(()->lift.extend(), lift)
        ).onFalse(
            new InstantCommand(()->lift.stop(), lift)
        );

        // Lift Down POV 180
        new POVButton(driverJoystick, 180).onTrue(
            new RunCommand(()->lift.retract(), lift)
        ).onFalse(
            new InstantCommand(()->lift.stop(), lift)
        );

        // Lift Down Dangerously (not respecting minimum setpoint) (13)
        new JoystickButton(driverJoystick, 13).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()->lift.setRespectMinimumSetpoint(false), lift),
                new RunCommand(()->lift.retract(), lift)
            )
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->lift.setRespectMinimumSetpoint(true), lift),
                new InstantCommand(()->lift.stop(), lift)
            )
        );

        // Arm Toward Back (5)
        new JoystickButton(driverJoystick, 5).onTrue(
            new RunCommand(()->arm.moveArmTowardBack(), arm)
        ).onFalse(
            new InstantCommand(()->arm.stop(), arm)
        );
       
        // Arm Toward Front (10)
        new JoystickButton(driverJoystick, 10).onTrue(
            new RunCommand(()->arm.moveArmTowardFront(), arm)
        ).onFalse(
            new InstantCommand(()->arm.stop(), arm)
        );;
       
        // Climber Down (7)

        // Climber Up (8)
        
        // OPERATOR BUTTONS
        // April Tag Lock Yaw (trigger)
        new JoystickButton(operatorJoystick, 1).onTrue(
            new InstantCommand(()->{
                this.isTurningToSeenAprilTagAngle = true;
            })
        ).onFalse(
            new InstantCommand(()->{
                this.isTurningToSeenAprilTagAngle = false;
            })
        );

        // Human Station Left Lock  (3)
        new JoystickButton(operatorJoystick, 3).onTrue(
            new InstantCommand(()->{
                this.driveTargetAngle = drivebase.isRedAlliance() ? 126.0 : 306.0;
                this.isTurningToAngle = true;
            })
        ).onFalse(
            new InstantCommand(()->{
                this.driveTargetAngle = 0.0;
                this.isTurningToAngle = false;
            })
        );

        // Human Station Right Lock Yaw (4)
        new JoystickButton(operatorJoystick, 4).onTrue(
            new InstantCommand(()->{
                this.driveTargetAngle = drivebase.isRedAlliance() ? 234.0 : 54.0;
                this.isTurningToAngle = true;
            })
        ).onFalse(
            new InstantCommand(()->{
                this.driveTargetAngle = 0.0;
                this.isTurningToAngle = false;
            })
        );

        // April Tag Lock Yaw With Left Offset (5)

        // April Tag Lock Yaw With Right Offset (6)
        
        // Send Lift to Level 3 (7)
        new JoystickButton(operatorJoystick, 7).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.goToLevel3(), lift),
                new InstantCommand(()->arm.goToScorePosition(), arm)
            )
        );

        // Send Lift to Level 2 (9)
        new JoystickButton(operatorJoystick, 9).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.goToLevel2(), lift),
                new InstantCommand(()->arm.goToScorePosition(), arm)
            )
        );

        // Send Lift to Level 1 (11)
        new JoystickButton(operatorJoystick, 11).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.goToLevel1(), lift),
                new InstantCommand(()->arm.goToScorePosition(), arm)
            )
        );

        // Send Lift to Receive Position (12)
        new JoystickButton(operatorJoystick, 12).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.goToRecieveSetpoint(), lift),
                new InstantCommand(()->arm.goToRecievePosition(), arm)
            )
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
