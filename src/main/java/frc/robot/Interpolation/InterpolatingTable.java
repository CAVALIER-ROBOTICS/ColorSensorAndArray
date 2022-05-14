// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;

/** Add your docs here. */
public class InterpolatingTable 
{
    private InterpolatingTable() {}

    // Interpolating tree map
    private static final TreeMap<Double, ShotParameter> map = new TreeMap<>(
        Map.ofEntries(
            // entry(.9, new ShotParameter(20, 1700, 0.0)),
            // entry(1.0, new ShotParameter(20, 1700, 0.0)),
            // entry(1.3, new ShotParameter(25, 1750, 0.0)),
            // entry(1.6, new ShotParameter(27, 1800, 0.0)),
            // entry(1.8, new ShotParameter(27, 1800, 0.0)),
            // entry(1.9, new ShotParameter(28, 1800, 0.0)),
            // entry(2.0, new ShotParameter(28, 1800, 0.0)),
            // entry(2.2, new ShotParameter(29, 1850, 0.0)), 
            // entry(2.4, new ShotParameter(30, 1900, 0.0)),
            // entry(2.6, new ShotParameter(30, 1950, 0.0)),
            // entry(2.8, new ShotParameter(32, 1950, 0.0)), 
            // entry(3.0, new ShotParameter(32, 1950, 0.0)),
            // entry(3.2, new ShotParameter(30, 2000, 0.0)),
            // entry(3.4, new ShotParameter(33.5, 2000, 0.0)),
            // entry(3.6, new ShotParameter(35, 2100, 0.0)), 
            // entry(3.8, new ShotParameter(36.5, 2200, 0.0)),
            // entry(4.0, new ShotParameter(37.7, 2200, 0.0)),
            // entry(4.3, new ShotParameter(40, 2200, 0.0)),
            // entry(4.5, new ShotParameter(38, 2540, 0.0))

            entry(1.08, new ShotParameter(22, .365, 0.0)),
            entry(1.4, new ShotParameter(28, .34, 0.0)),
            entry(1.72, new ShotParameter(29, .41, 0.0)),
            entry(2.08, new ShotParameter(30, .413, 0.0)),
            entry(2.5, new ShotParameter(31, .43, 0.0)),
            entry(2.9, new ShotParameter(31, .45, 0.0)),  
            entry(3.1, new ShotParameter(38, .453, 0.0)),
            entry(3.2, new ShotParameter(39, .453, 0.0)),
            entry(3.7, new ShotParameter(39.5, .463, 0.0)),
            entry(4.0, new ShotParameter(40, .4689, 0.0)),
            entry(4.11, new ShotParameter(40, .47, 0.0)),
            entry(5.0, new ShotParameter(44, .52, 0.0))
            // entry(30.0, new ShotParameter(44, .6, 0.0))
        )
        
    );

    public static ShotParameter getShotParameter(double distance)
    {
        Entry<Double, ShotParameter> ceilEntry = map.ceilingEntry(distance);
        Entry<Double, ShotParameter> floorEntry = map.floorEntry(distance);
        if (ceilEntry == null)
        {
            return floorEntry.getValue();
        } 

        if (floorEntry == null)
        {
            return ceilEntry.getValue();
        }

        if (ceilEntry.getValue().equals(floorEntry.getValue()))
        {
            return ceilEntry.getValue();
        } 
        
        return ceilEntry.getValue().interpolate(
            floorEntry.getValue(), 
            (distance - floorEntry.getKey())/(ceilEntry.getKey() - floorEntry.getKey())
        );
    } 
}
