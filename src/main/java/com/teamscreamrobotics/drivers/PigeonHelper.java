package com.teamscreamrobotics.drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.AngularVelocity;

public class PigeonHelper {
    // Status signals for velocity data
    private StatusSignal<AngularVelocity> velocityX;
    private StatusSignal<AngularVelocity> velocityY;
    private StatusSignal<AngularVelocity> velocityZ;
    
    public PigeonHelper(Pigeon2 pigeon) {
        // Get velocity status signals
        velocityX = pigeon.getAngularVelocityXWorld();
        velocityY = pigeon.getAngularVelocityYWorld();
        velocityZ = pigeon.getAngularVelocityZWorld();
    }
    
    /**
     * Get the angular velocity around the X axis in degrees per second
     */
    public double getVelocityX() {
        velocityX.refresh();
        return velocityX.getValueAsDouble();
    }
    
    /**
     * Get the angular velocity around the Y axis in degrees per second
     */
    public double getVelocityY() {
        velocityY.refresh();
        return velocityY.getValueAsDouble();
    }
    
    /**
     * Get the angular velocity around the Z axis (yaw rate) in degrees per second
     */
    public double getVelocityZ() {
        velocityZ.refresh();
        return velocityZ.getValueAsDouble();
    }
    
    /**
     * Get all velocity values at once
     * Returns array: [velocityX, velocityY, velocityZ]
     */
    public double[] getAllVelocities() {
        // Refresh all signals together for better performance
        velocityX.refresh();
        velocityY.refresh();
        velocityZ.refresh();
        
        return new double[] {
            velocityX.getValueAsDouble(),
            velocityY.getValueAsDouble(),
            velocityZ.getValueAsDouble()
        };
    }
    
    /**
     * Get the yaw rate (most commonly used for drivetrain)
     */
    public double getYawRate() {
        return getVelocityZ();
    }
    
    /**
     * Get yaw rate in radians per second (useful for some calculations)
     */
    public double getYawRateRadians() {
        velocityZ.refresh();
        return Math.toRadians(velocityZ.getValueAsDouble());
    }
}
