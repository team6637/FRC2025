// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareToScore extends SequentialCommandGroup {
    private final Lift lift = new Lift();
    private final Arm arm = new Arm();
  /** Creates a new PrepareToScore. */
  public PrepareToScore() {
    addCommands(
        new InstantCommand(()->lift.goToLevel3(), lift),
        new InstantCommand(()->arm.goToScorePosition(), arm)
    );
        

  }
}
