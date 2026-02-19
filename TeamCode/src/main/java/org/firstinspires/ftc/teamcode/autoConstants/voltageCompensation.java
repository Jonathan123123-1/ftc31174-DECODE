package org.firstinspires.ftc.teamcode.autoConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * voltageCompensation
 *
 * Automatically adjusts motor power based on battery voltage levels.
 * This ensures that autonomous paths remain consistent whether the
 * battery is at 14V or 11V.
 */
public class voltageCompensation {

    // The voltage level that your robot was tuned at (usually 12V or 12.5V)
    public static final double NOMINAL_VOLTAGE = 12.5;

    /**
     * Calculates a compensation multiplier based on current battery voltage.
     * Multiplier = Nominal Voltage / Current Voltage
     * 
     * Example: 
     * If battery is 10V and Nominal is 12V, multiplier is 1.2x.
     * If battery is 14V and Nominal is 12V, multiplier is 0.85x.
     */
    public static double getVoltageMultiplier(HardwareMap hardwareMap) {
        double currentVoltage = getBatteryVoltage(hardwareMap);
        
        // Prevent division by zero and extreme values
        if (currentVoltage < 8.0) return 1.0; 
        
        return NOMINAL_VOLTAGE / currentVoltage;
    }

    /**
     * Gets the current battery voltage from the hardware map.
     */
    public static double getBatteryVoltage(HardwareMap hardwareMap) {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result == Double.POSITIVE_INFINITY ? NOMINAL_VOLTAGE : result;
    }
}
