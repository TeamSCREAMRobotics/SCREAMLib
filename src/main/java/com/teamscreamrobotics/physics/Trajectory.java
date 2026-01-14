package com.teamscreamrobotics.physics;

import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

/**
 * Trajectory calculator for projectile motion with air resistance
 * Designed for scoring with a ball
 * 
 * Usage:
 *   Trajectory.configure()
 *       .setShotVelocity(12.0)
 *       .setShotAngle(45.0)
 *       .setTargetDistance(4.0)
 *       .setGamePiece(GamePiece.FUEL);
 *   
 *   List<TrajectoryPoint> path = Trajectory.getDesiredTrajectory();
 *   double angle = Trajectory.getOptimalAngle();
 */
public class Trajectory {
    //if you read this, I went insane making this, enjoy.
    
    // Physical constants
    private static final double GRAVITY = 9.81; // m/s^2
    private static final double AIR_DENSITY = 1.225; // kg/m^3 at sea level
    
    // HUB specifications from manual
    public static final double HUB_HEIGHT = Units.inchesToMeters(72.0); // Opening front edge height
    public static final double HUB_DISTANCE_FROM_ALLIANCE_WALL = Units.inchesToMeters(158.6);
    
    // Static configuration instance
    private static TrajectoryConfig config = new TrajectoryConfig();
    
    /**
     * Game piece types with their physical properties
     */
    public enum GamePiece {
        FUEL(0.215, 0.150, 0.47),     // REBUILT FUEL: 0.448-0.500 lbs, 5.91" diameter
        CUSTOM(0.200, 0.150, 0.47);   // Custom game piece - use setCustomGamePiece() to configure
        
        public final double mass;      // kg
        public final double diameter;  // meters
        public final double dragCoefficient;
        
        GamePiece(double mass, double diameter, double dragCoefficient) {
            this.mass = mass;
            this.diameter = diameter;
            this.dragCoefficient = dragCoefficient;
        }
        
        public double getArea() {
            return Math.PI * Math.pow(diameter / 2, 2);
        }
    }
    
    /**
     * Configuration builder for trajectory calculations
     */
    public static class TrajectoryConfig {
        private double shotVelocity = 10.0;      // m/s
        private double shotAngle = 45.0;         // degrees
        private double robotVelocity = 0.0;      // m/s
        private double robotAngle = 0.0;         // degrees
        private double targetDistance = 4.0;     // meters
        private double targetHeight = HUB_HEIGHT;// meters
        private double initialHeight = 0.5;      // meters
        private GamePiece gamePiece = GamePiece.FUEL;
        
        // Custom game piece properties (used when gamePiece == CUSTOM)
        private double customMass = 0.200;
        private double customDiameter = 0.150;
        private double customDragCoeff = 0.47;
        
        public TrajectoryConfig setShotVelocity(double velocity) {
            this.shotVelocity = velocity;
            return this;
        }
        
        public TrajectoryConfig setShotAngle(double angle) {
            this.shotAngle = angle;
            return this;
        }
        
        public TrajectoryConfig setRobotVelocity(double velocity) {
            this.robotVelocity = velocity;
            return this;
        }
        
        public TrajectoryConfig setRobotAngle(double angle) {
            this.robotAngle = angle;
            return this;
        }
        
        public TrajectoryConfig setTargetDistance(double distance) {
            this.targetDistance = distance;
            return this;
        }
        
        public TrajectoryConfig setTargetHeight(double height) {
            this.targetHeight = height;
            return this;
        }
        
        public TrajectoryConfig setInitialHeight(double height) {
            this.initialHeight = height;
            return this;
        }
        
        public TrajectoryConfig setGamePiece(GamePiece piece) {
            this.gamePiece = piece;
            return this;
        }
        
        /**
         * Set custom game piece properties
         * @param massKg Mass in kilograms
         * @param diameterMeters Diameter in meters
         * @param dragCoefficient Drag coefficient (sphere ≈ 0.47)
         */
        public TrajectoryConfig setCustomGamePiece(double massKg, double diameterMeters, double dragCoefficient) {
            this.customMass = massKg;
            this.customDiameter = diameterMeters;
            this.customDragCoeff = dragCoefficient;
            this.gamePiece = GamePiece.CUSTOM;
            return this;
        }
        
        /**
         * Convenience method to set custom game piece from imperial units
         * @param massLbs Mass in pounds
         * @param diameterInches Diameter in inches
         * @param dragCoefficient Drag coefficient
         */
        public TrajectoryConfig setCustomGamePieceImperial(double massLbs, double diameterInches, double dragCoefficient) {
            this.customMass = massLbs * 0.453592; // lbs to kg
            this.customDiameter = diameterInches * 0.0254; // inches to meters
            this.customDragCoeff = dragCoefficient;
            this.gamePiece = GamePiece.CUSTOM;
            return this;
        }
        
        private double getMass() {
            return gamePiece == GamePiece.CUSTOM ? customMass : gamePiece.mass;
        }
        
        private double getDiameter() {
            return gamePiece == GamePiece.CUSTOM ? customDiameter : gamePiece.diameter;
        }
        
        private double getDragCoefficient() {
            return gamePiece == GamePiece.CUSTOM ? customDragCoeff : gamePiece.dragCoefficient;
        }
        
        private double getArea() {
            double d = getDiameter();
            return Math.PI * Math.pow(d / 2, 2);
        }
    }
    
    /**
     * Represents a point in the trajectory
     */
    public static class TrajectoryPoint {
        public final double time;
        public final double x;
        public final double y;
        public final double vx;
        public final double vy;
        
        public TrajectoryPoint(double time, double x, double y, double vx, double vy) {
            this.time = time;
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
        }
        
        @Override
        public String toString() {
            return String.format("t=%.3f, x=%.3f, y=%.3f, vx=%.3f, vy=%.3f", 
                                time, x, y, vx, vy);
        }
    }
    
    /**
     * Get the configuration builder
     */
    public static TrajectoryConfig configure() {
        return config;
    }
    
    /**
     * Get the current configuration (read-only access)
     */
    public static TrajectoryConfig getConfig() {
        return config;
    }
    
    /**
     * Main method to get the desired trajectory based on current configuration
     * @return List of trajectory points
     */
    public static List<TrajectoryPoint> getDesiredTrajectory() {
        return calculateTrajectory(
            config.shotVelocity,
            config.shotAngle,
            config.robotVelocity,
            config.robotAngle,
            config.targetDistance,
            config.initialHeight,
            config.getMass(),
            config.getArea(),
            config.getDragCoefficient()
        );
    }
    
    /**
     * Get the optimal angle to hit the configured target
     * @return Optimal angle in degrees, or -1 if no solution found
     */
    public static double getOptimalAngle() {
        return findOptimalAngle(
            config.shotVelocity,
            config.robotVelocity,
            config.robotAngle,
            config.targetDistance,
            config.targetHeight,
            config.initialHeight,
            config.getMass(),
            config.getArea(),
            config.getDragCoefficient()
        );
    }
    
    /**
     * Get the required velocity to hit the configured target at the configured angle
     * @return Required velocity in m/s, or -1 if no solution found
     */
    public static double getRequiredVelocity() {
        return calculateRequiredVelocity(
            config.shotAngle,
            config.robotVelocity,
            config.robotAngle,
            config.targetDistance,
            config.targetHeight,
            config.initialHeight,
            config.getMass(),
            config.getArea(),
            config.getDragCoefficient()
        );
    }
    
    /**
     * Get the time of flight for the configured shot
     * @return Time in seconds
     */
    public static double getTimeOfFlight() {
        List<TrajectoryPoint> traj = getDesiredTrajectory();
        
        double closestTime = 0;
        double minDist = Double.MAX_VALUE;
        
        for (TrajectoryPoint p : traj) {
            double dist = Math.abs(p.x - config.targetDistance);
            if (dist < minDist) {
                minDist = dist;
                closestTime = p.time;
            }
        }
        
        return closestTime;
    }
    
    /**
     * Internal trajectory calculation with air resistance
     */
    private static List<TrajectoryPoint> calculateTrajectory(
            double shotVelocity,
            double shotAngle,
            double robotVelocity,
            double robotAngle,
            double distance,
            double initialHeight,
            double mass,
            double area,
            double dragCoeff) {
        
        List<TrajectoryPoint> points = new ArrayList<>();
        
        // Convert angles to radians
        double shotAngleRad = Math.toRadians(shotAngle);
        double robotAngleRad = Math.toRadians(robotAngle);
        
        // Initial velocity components (shot + robot motion)
        double vx0 = shotVelocity * Math.cos(shotAngleRad) + 
                     robotVelocity * Math.cos(robotAngleRad);
        double vy0 = shotVelocity * Math.sin(shotAngleRad) + 
                     robotVelocity * Math.sin(robotAngleRad);
        
        // Simulation parameters
        double dt = 0.001; // Time step (1ms)
        double maxTime = 5.0; // Maximum simulation time
        
        // Initial state
        double t = 0;
        double x = 0;
        double y = initialHeight;
        double vx = vx0;
        double vy = vy0;
        
        // Add initial point
        points.add(new TrajectoryPoint(t, x, y, vx, vy));
        
        // Numerical integration using Euler method
        while (t < maxTime && y >= 0) {
            // Calculate air resistance
            double v = Math.sqrt(vx * vx + vy * vy);
            double dragForce = 0.5 * AIR_DENSITY * dragCoeff * area * v * v;
            
            // Acceleration due to drag (opposite to velocity)
            double ax = -(dragForce / mass) * (vx / v);
            double ay = -GRAVITY - (dragForce / mass) * (vy / v);
            
            // Update velocity
            vx += ax * dt;
            vy += ay * dt;
            
            // Update position
            x += vx * dt;
            y += vy * dt;
            
            // Update time
            t += dt;
            
            // Store point every 10ms for efficiency
            if ((int)(t * 100) % 1 == 0) {
                points.add(new TrajectoryPoint(t, x, y, vx, vy));
            }
        }
        
        return points;
    }
    
    /**
     * Internal optimal angle finder
     */
    private static double findOptimalAngle(
            double shotVelocity,
            double robotVelocity,
            double robotAngle,
            double targetDistance,
            double targetHeight,
            double initialHeight,
            double mass,
            double area,
            double dragCoeff) {
        
        double bestAngle = -1;
        double minError = Double.MAX_VALUE;
        
        // Search for best angle (coarse)
        for (double angle = 10; angle <= 80; angle += 1.0) {
            List<TrajectoryPoint> traj = calculateTrajectory(
                shotVelocity, angle, robotVelocity, robotAngle, 
                targetDistance, initialHeight, mass, area, dragCoeff);
            
            for (TrajectoryPoint p : traj) {
                double distError = Math.abs(p.x - targetDistance);
                double heightError = Math.abs(p.y - targetHeight);
                double totalError = distError + heightError;
                
                if (totalError < minError) {
                    minError = totalError;
                    bestAngle = angle;
                }
            }
        }
        
        // Fine-tune
        if (bestAngle > 0) {
            for (double angle = bestAngle - 1; angle <= bestAngle + 1; angle += 0.1) {
                List<TrajectoryPoint> traj = calculateTrajectory(
                    shotVelocity, angle, robotVelocity, robotAngle, 
                    targetDistance, initialHeight, mass, area, dragCoeff);
                
                for (TrajectoryPoint p : traj) {
                    double distError = Math.abs(p.x - targetDistance);
                    double heightError = Math.abs(p.y - targetHeight);
                    double totalError = distError + heightError;
                    
                    if (totalError < minError) {
                        minError = totalError;
                        bestAngle = angle;
                    }
                }
            }
        }
        
        return bestAngle;
    }
    
    /**
     * Internal required velocity calculator
     */
    private static double calculateRequiredVelocity(
            double shotAngle,
            double robotVelocity,
            double robotAngle,
            double targetDistance,
            double targetHeight,
            double initialHeight,
            double mass,
            double area,
            double dragCoeff) {
        
        double bestVelocity = -1;
        double minError = Double.MAX_VALUE;
        
        // Search for best velocity (coarse)
        for (double velocity = 1.0; velocity <= 30.0; velocity += 0.5) {
            List<TrajectoryPoint> traj = calculateTrajectory(
                velocity, shotAngle, robotVelocity, robotAngle, 
                targetDistance, initialHeight, mass, area, dragCoeff);
            
            for (TrajectoryPoint p : traj) {
                double distError = Math.abs(p.x - targetDistance);
                double heightError = Math.abs(p.y - targetHeight);
                double totalError = distError + heightError;
                
                if (totalError < minError) {
                    minError = totalError;
                    bestVelocity = velocity;
                }
            }
        }
        
        // Fine-tune
        if (bestVelocity > 0) {
            for (double velocity = bestVelocity - 0.5; velocity <= bestVelocity + 0.5; velocity += 0.05) {
                List<TrajectoryPoint> traj = calculateTrajectory(
                    velocity, shotAngle, robotVelocity, robotAngle, 
                    targetDistance, initialHeight, mass, area, dragCoeff);
                
                for (TrajectoryPoint p : traj) {
                    double distError = Math.abs(p.x - targetDistance);
                    double heightError = Math.abs(p.y - targetHeight);
                    double totalError = distError + heightError;
                    
                    if (totalError < minError) {
                        minError = totalError;
                        bestVelocity = velocity;
                    }
                }
            }
        }
        
        return bestVelocity;
    }
}
