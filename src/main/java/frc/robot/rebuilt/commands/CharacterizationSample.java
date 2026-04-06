package frc.robot.rebuilt.commands;

/**
 * A single high-frequency characterization sample collected at ~250 Hz.
 *
 * @param timestampSeconds FPGA timestamp in seconds when the sample was taken
 * @param positionRot turret position in mechanism rotations
 * @param velocityRotPerSec turret velocity in mechanism rot/s
 * @param accelerationRotPerSecSq turret acceleration in mechanism rot/s^2 (from TalonFX signal)
 * @param currentAmps measured torque current in Amps (from TalonFX signal)
 */
public record CharacterizationSample(
    double timestampSeconds,
    double positionRot,
    double velocityRotPerSec,
    double accelerationRotPerSecSq,
    double currentAmps) {}
