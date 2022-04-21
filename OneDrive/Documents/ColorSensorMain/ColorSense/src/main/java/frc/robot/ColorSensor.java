package frc.robot;

import java.util.ArrayList;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.IntakeSubsystem;

public class ColorSensor {
    public static enum colorStates {
        RED,
        BLUE,
        UNDEFINED
    }  

    private static final I2C.Port i2cport = I2C.Port.kOnboard;
    public static ColorSensorV3 colorSensorV3 = new ColorSensorV3(i2cport);
    private static final ColorMatch cMatch = new ColorMatch();
    
    static final Color kBlueTarget = new Color(0,0,241);
    static final Color kRedTarget = new Color(241,0,0);

    
    static colorStates lastColor = colorStates.UNDEFINED;


    public static ArrayList<colorStates> intakeState = new ArrayList<>();
    public static void updateLastColor() {
        SmartDashboard.putBoolean("intaking", IntakeSubsystem.isIntaking());
        SmartDashboard.putBoolean("outtaking", IntakeSubsystem.isOuttaking());
        Color detectedColor = colorSensorV3.getColor();
        colorStates cState;
        cMatch.addColorMatch(kBlueTarget);
        cMatch.addColorMatch(kRedTarget);
        ColorMatchResult matchResult = cMatch.matchClosestColor(detectedColor);
        //int proxy = colorSensorV3.getProximity();
    
        if((matchResult.color == kBlueTarget)&&(detectedColor.blue>.3)) {
            cState = colorStates.BLUE;
        }
        else if((matchResult.color == kRedTarget)&&(detectedColor.red>.4)) {
            cState = colorStates.RED;
        }
        else {
            cState = colorStates.UNDEFINED;
        }

        if(cState != lastColor) {
            lastColor = cState;
            if(cState != colorStates.UNDEFINED) {
                updateBallState();
            }
        }

        if (lastColor == colorStates.RED) {
            SmartDashboard.putString("color", "red");
        }
        else if(lastColor == colorStates.BLUE) {
            SmartDashboard.putString("color", "blue");
        }
        else{
            SmartDashboard.putString("color", "undefined");
        }
    
    }

    public static void updateBallState() {
      
        if(IntakeSubsystem.isIntaking()) {
            intakeState.add(lastColor);
        }
        else if(IntakeSubsystem.isOuttaking()) {
            if(intakeState.size()>1) {
               removeBall(1);
        }
            else if(intakeState.size()<1) {
                removeBall(0);
            }
           
               System.out.println(intakeState);
            
        }

        if(intakeState.size()>2) {
            SmartDashboard.putBoolean("INTAKESTATE_FAIL", true);
        }
    }

    public static void addBall(colorStates colorState) {
        intakeState.add(colorState);
    }

    public static void removeBall(int index) {
        intakeState.remove(index);
    }

    public static ArrayList<colorStates> getBallState() {
        return intakeState;
    }

    public static colorStates getAllianceColor() {
        // if (DriverStation.getAlliance() == Alliance.Red) {
        //     return colorStates.RED;
        // }
        // else{
            return colorStates.BLUE;
       // }
    }
}
