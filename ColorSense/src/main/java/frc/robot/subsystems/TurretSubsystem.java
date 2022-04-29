// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  CANSparkMax turret = new CANSparkMax(Constants.turretID, MotorType.kBrushless);
  RelativeEncoder encoder = turret.getEncoder();
  SparkMaxPIDController turretPID = turret.getPIDController();
  public int acceptedVolts = 30;
  public boolean hasHomed;

  public double kP, kI, kD, kIz, kFF;
  
  boolean turnUp;
  boolean turnDown;

  double desiredAngle;
  
  public TurretSubsystem() 
  {
    turret.restoreFactoryDefaults();
    turret.setIdleMode(IdleMode.kBrake);
    turret.setInverted(false);
    // turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    // turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    
    // turret.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 41);
    // turret.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);
    turret.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65000);
    turret.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65000);

    kP = 0.09; 
    kI = 0.000;
    kD = 0; 
    kIz = 0; 
    kFF =  0; //0.001; 

    // set PID coefficients
    turretPID.setP(kP);
    turretPID.setI(kI);
    turretPID.setD(kD);
    turretPID.setIZone(kIz);
    turretPID.setFF(kFF);


    turnUp = false;
    turnDown = false;

    turret.setOpenLoopRampRate(0.7);

    hasHomed = false;
  }

  public void softLimit(boolean x)
  {
    // turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, x);
    // turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, x);
  }

  public double getPos() {
    return encoder.getPosition();
  }

  public double getAngle() {
    return  (getPos() / 50) * 360;
  }

  public void setDesiredAngle(double angle) {
    turretPID.setReference(angle/360*50, ControlType.kPosition);
  }

  public void setOpenLoop(double volts) {
    turretPID.setReference(volts, ControlType.kDutyCycle);
  }

  public void resetPos() {
    encoder.setPosition(0.0);
  }

  public boolean inRange() {
    return (getAngle()>90 || getAngle()<213);
  }

  public void updateTurns()
  {
    if(getAngle()<10) {turnUp = true;}
    
    if(getAngle()>290) {turnDown = true;}

    if(getAngle()>250&& turnUp) {turnUp = false;}

    if(getAngle()<50 && turnDown) {turnDown = false;}
  }

  public void aim()
  {
    updateTurns();
    if(inRange()&&!turnDown&&!turnUp) {
      setDesiredAngle(desiredAngle);
    }
    else {
      if(turnUp) { 
        setOpenLoop(0.3);
      }
      if(turnDown) {
        setOpenLoop(-0.3);
      }
    }
  }


  public void startTurret() {
    setOpenLoop(-.35);
    if(getVolts()>65) {
      hasHomed = true;
      resetPos();
    }
  }

  public boolean getHasHomed() {
    return hasHomed;
  }

  // public void manualAim(double volt) {
  //   if(inRange()) {
  //     setOpenLoop(volt);
  //   }
  //   else {
  //     if(getAngle()>290 && volt<0) { 
  //       setOpenLoop(volt);
  //     }
  //     if(getAngle()<10 && volt>0) {
  //       setOpenLoop(volt);
  //     }    
  //   }
  //   SmartDashboard.putNumber("Manual aim angle", volt);
  // }

 public void setVolts(double x) {
   turret.setVoltage(x);
 }

  public void manualAimUp() {
    if(getAngle()<280) {
      setVolts(4);
    }
  }

  public void manualAimDown() {
    if(getAngle()>20) {
      setVolts(-4);
    }
  }

  public void stop() {
      setOpenLoop(0);
  }

  public double getVolts() {
    return turret.getOutputCurrent();
  }

  @Override
  public void periodic() {
    if(Limelight.getCurrentMode() == Limelight.ledModes.ON) {
      desiredAngle = getAngle() + Limelight.getX();
      }
      else {
        desiredAngle = 166.9;
      }
  }
}
