// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.HopperFloorSubsystem;
import frc.robot.subsystems.KickerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class KickCommand extends ParallelCommandGroup {
  /** Creates a new KickCommand. */
  public KickCommand(KickerSubsystem kickSub, HopperFloorSubsystem floorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StartEndCommand(
        ()-> kickSub.setKicker(8), 
        ()-> kickSub.stopKicker(),
        kickSub),
      new StartEndCommand(
        () -> floorSub.setFloor(4), 
        () -> floorSub.stopFloor(),
        floorSub)
    );
  }
}
