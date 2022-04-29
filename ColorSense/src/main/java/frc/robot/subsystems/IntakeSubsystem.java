// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeID,MotorType.kBrushless);

  public static boolean intakingBoolean = false;
  public static boolean outtakingBoolean = false;

  public IntakeSubsystem() 
  {
    intakeMotor.setOpenLoopRampRate(2);
   // raiseEnc.setPosition(0);
   intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65000);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 64900);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 64800);
  }

  public void setIntakeMotor(double x)
  {
    intakeMotor.setVoltage(x);
    if(x>0) {
      intakingBoolean = true;
    }
    else if(x<0) {
      outtakingBoolean = true;
    }
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
    
    intakingBoolean = false;
    outtakingBoolean = false;
  }

  public static boolean isIntaking() {
    return intakingBoolean;
  }

  public static boolean isOuttaking() {
    return outtakingBoolean;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Raise Encoder Value", getRaiseEnc());
  }
}
