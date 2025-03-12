// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Lift;

public class PrepareToScore extends SequentialCommandGroup {

    public PrepareToScore(Arm arm, Lift lift, CoralIntake coralIntake) {
        addCommands(
            new InstantCommand(()->lift.goToLevel4(), lift),
            new InstantCommand(()->arm.goToLevel4Position(), arm),
            new InstantCommand(()->coralIntake.inSoftly(), coralIntake)
        );
    }
}
