package com.team4522.lib.sim;

public record SimState(double position, double velocity, double supplyVoltage){
        /**
         * Gets the position from the simulation.
         * 
         * The units for each simulation type are as follows:
         * <ul>
         *     <li>DCMotorSim: Angular Position (Rotations)</li>
         *     <li>ElevatorSim: Linear Position (Meters)</li>
         *     <li>SingleJointedArmSim: Angular Position (Rotations)</li>
         *     <li>FlywheelSim: N/A</li>
         * </ul>
         * 
         */
        @Override
        public double position(){
            return position;
        }

            /**
         * Gets the velocity from the simulation.
         * 
         * The units for each simulation type are as follows:
         * <ul>
         *     <li>DCMotorSim: Angular Velocity (rot/s)</li>
         *     <li>ElevatorSim: Linear Velocity (m/s)</li>
         *     <li>SingleJointedArmSim: Angular Velocity (rot/s)</li>
         *     <li>FlywheelSim: Angular Velocity (rot/s)</li>
         * </ul>
         * 
         */
        @Override
        public double velocity(){
            return velocity;
        }
    }
