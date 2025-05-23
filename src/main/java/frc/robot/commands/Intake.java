// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Lift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake extends SequentialCommandGroup {
  public Intake(Arm arm, CoralIntake coralIntake, Lift lift) {
    addCommands(
        new WaitUntilCommand(()->lift.atSetpoint()),
        new WaitUntilCommand(()->arm.atSetpoint()),
        new ParallelRaceGroup(
            new RunCommand(()->coralIntake.collect(), coralIntake),
            new WaitCommand(1) 
        ),
        new InstantCommand(()->coralIntake.stop(), coralIntake)
    );
  }
}
