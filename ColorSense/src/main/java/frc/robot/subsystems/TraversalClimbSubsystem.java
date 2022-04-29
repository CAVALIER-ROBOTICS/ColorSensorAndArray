// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TraversalClimbSubsystem extends SubsystemBase {
  /** Creates a new TraversalClimbSubsystem. */
  private CANSparkMax climb = new CANSparkMax(Constants.trasversalID, MotorType.kBrushless);
  private RelativeEncoder climbEnc = climb.getEncoder();
  private SparkMaxPIDController climbPID = climb.getPIDController();
  
  public static double climbEncValue;

  public TraversalClimbSubsystem() {
    climb.restoreFactoryDefaults();
    climb.setIdleMode(IdleMode.kBrake);
    climb.setInverted(false);
    climbPID.setP(0.03);
    climbPID.setI(0);
    climbPID.setD(0);

    climb.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 61000);
    climb.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 62100);
    // climbPID.setOutputRange(-.2, .2);

    climb.setOpenLoopRampRate(.5);

    climbEnc.setPosition(0);

  }

  public void setClimb(double x) {
    climb.setVoltage(x);
  }

  // public void resetPos() {
  //   climbEnc.setPosition(0);
  // }

  public void setClimbPos() {
    climbPID.setReference(climbEnc.getPosition(), ControlType.kPosition);
  }

  public double getClimbPos() {
    return climbEnc.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Traversal Climb enc", getClimbPos());
    climbEncValue = climbEnc.getPosition();
  }
}
