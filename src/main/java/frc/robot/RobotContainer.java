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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    Joystick driverJoystick = new Joystick(0);
    Joystick operatorJoystick = new Joystick(1);
    private final Limelight limelight = new Limelight("limelight");
    private final Lift lift = new Lift();
    private final Arm arm = new Arm();
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final CoralIntake coralIntake = new CoralIntake();
    //private final Climber climber = new Climber();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    private boolean isTurningToAngle = false;
    private double driveTargetAngle = 0.0;
    private boolean isTurningToSeenAprilTagAngle = false;
    private boolean isFieldOriented = true;

    public RobotContainer() {
        drivebase.setDefaultCommand(
            new TeleopDriveCommand(
                drivebase, 
                ()-> -MathUtil.applyDeadband(driverJoystick.getY(), OperatorConstants.Y_DEADBAND),
                ()-> -MathUtil.applyDeadband(driverJoystick.getX(), OperatorConstants.Y_DEADBAND),
                ()-> -MathUtil.applyDeadband(driverJoystick.getTwist(), OperatorConstants.TWIST_DEADBAND),
                limelight,
                ()->isTurningToAngle,
                ()->driveTargetAngle,
                ()->isTurningToSeenAprilTagAngle,
                ()->isFieldOriented
            )
        ); 

        configureBindings();
    }

    private void configureBindings() {
        // DRIVER BUTTONS

        // DRIVE IN ROBOT RELATIVE MODE
        new JoystickButton(driverJoystick, 2).onTrue(
            new InstantCommand(()->{
                this.isFieldOriented = false;
            })
        ).onFalse(
            new InstantCommand(()->{
                this.isFieldOriented = true;
            })
        );

        // Intake In and Algae In (3)
        new JoystickButton(driverJoystick, 3).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->coralIntake.collect(), coralIntake),
                new InstantCommand(()->algaeIntake.intake(), algaeIntake)
            ) 
        ).onFalse(
            new ParallelCommandGroup(
                new InstantCommand(()->coralIntake.stop(), coralIntake),
                new InstantCommand(()->algaeIntake.stopIntake(), algaeIntake)
                )
            );

        // Intake Out and Algae Out (1)
        new JoystickButton(driverJoystick, 1).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->algaeIntake.score(), algaeIntake),
                new InstantCommand(()->coralIntake.score(), coralIntake)
            )
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->coralIntake.stop(), coralIntake),
                new InstantCommand(()->algaeIntake.stopIntake(), algaeIntake),
            new ConditionalCommand(
                new InstantCommand(()->arm.backOff(), arm),
                new InstantCommand(), 
                ()->(lift.getPosition() > 200)
                )
            )
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
        );
        

        // Reset Yaw (14)
        new JoystickButton(driverJoystick, 14).onTrue(
            new InstantCommand(()->drivebase.zeroGyro(), drivebase)
        );

        // Algae Eject High (6)
        new JoystickButton(driverJoystick, 6).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.setSetpoint(170), lift),
                new InstantCommand(()->arm.setSetpoint(120), arm)
            )
        );
        // Algae Eject Low (9)
        new JoystickButton(driverJoystick, 9).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.setSetpoint(90), lift),
                new InstantCommand(()->arm.setSetpoint(120), arm)
            )
        );
       
        // Climber Down (7)
        // new JoystickButton(driverJoystick, 7).onTrue(
        //     new RunCommand(()-> climber.down(), climber)
        // ).onFalse(
        //         new InstantCommand(()->climber.stop(), climber)
        // );

        // // Climber Up (8)
        // new JoystickButton(driverJoystick, 8).onTrue(
        //     new RunCommand(()-> climber.up(), climber)
        // ).onFalse(
        //     new InstantCommand(()->climber.stop(), climber)
        // );

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
                this.driveTargetAngle = drivebase.isRedAlliance() ? 306.0 : 126.0;
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
                this.driveTargetAngle = drivebase.isRedAlliance() ? 54.0 : 234.0;
                this.isTurningToAngle = true;
            })
        ).onFalse(
            new InstantCommand(()->{
                this.driveTargetAngle = 0.0;
                this.isTurningToAngle = false;
            })
        );

        // April Tag Lock Yaw With Left Offset (5)
        new JoystickButton(operatorJoystick, 5).whileTrue(
            drivebase.driveToReef(false)
        );


        // April Tag Lock Yaw With Right Offset (6)
        // new JoystickButton(operatorJoystick, 6).whileTrue(
        //     drivebase.driveToReef(true)
        // );

        // send lift to start position(10)
        new JoystickButton(operatorJoystick, 10).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()->lift.goToStart(), lift),
                new WaitUntilCommand(()->lift.atSetpoint()),
                new InstantCommand(()->arm.goToStartingPosition(), arm)
            )
        );
        
        // Send Lift to Level 4 (7)
        new JoystickButton(operatorJoystick, 7).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.goToLevel4(), lift),
                new InstantCommand(()->arm.goToLevel4Position(), arm)
            )
        );

        // Send Lift to Level 3 (9)
        new JoystickButton(operatorJoystick, 9).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.goToLevel3(), lift),
                new InstantCommand(()->arm.goToLevel3Position(), arm)
            )
        );

        // Send Lift to Level 2 (11)
        new JoystickButton(operatorJoystick, 11).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(()->lift.goToLevel2(), lift),
                new InstantCommand(()->arm.goToLevel2Position(), arm)
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
