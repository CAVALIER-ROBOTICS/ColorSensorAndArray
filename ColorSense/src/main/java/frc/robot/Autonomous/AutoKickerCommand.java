// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.KickerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoKickerCommand extends SequentialCommandGroup {
  /** Creates a new AutoKickerCommand. */
  public AutoKickerCommand(KickerSubsystem kickerSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(2),
      new InstantCommand(()-> kickerSub.setKicker(10)),
      new WaitCommand(4),
      new InstantCommand(()-> kickerSub.setKicker(0)),
      new WaitCommand(11),
      new InstantCommand(()-> kickerSub.setKicker(3))
    );
  }
}
