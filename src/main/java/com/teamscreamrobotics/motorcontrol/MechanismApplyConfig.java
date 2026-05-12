package com.teamscreamrobotics.motorcontrol;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.teamscreamrobotics.power.PowerPriority;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Distance;

/**
 * Bundles all runtime settings passed from a mechanism config to
 * {@link TalonFXWrapper#applyMechanismConfig}.  Package-private; not user-facing.
 */
record MechanismApplyConfig(
        String logPrefix,
        TalonFXConfiguration talonFXConfig,
        TalonFXConfiguration simTalonFXConfig,          // nullable
        CANcoderConfiguration cancoderConfig,            // nullable — no CANcoder when null
        int cancoderCanId,
        String cancoderCanbus,
        boolean seedFromCANcoder,
        Distance mechanismCircumference,                 // nullable
        ArmFeedforward armFeedforward,                   // nullable
        ArmFeedforward simArmFeedforward,                // nullable
        ElevatorFeedforward elevatorFeedforward,         // nullable
        ElevatorFeedforward simElevatorFeedforward,      // nullable
        SimpleMotorFeedforward simpleFeedforward,        // nullable
        SimpleMotorFeedforward simSimpleFeedforward,     // nullable
        double horizontalZeroRad,
        PowerPriority powerPriority,
        int[] pdhChannels) {}
