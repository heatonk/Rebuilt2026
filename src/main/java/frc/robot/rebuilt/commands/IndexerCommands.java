package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.util.LedStrip;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;

public class IndexerCommands {
  /** declares variables that will later hold state objects */
  private Map<String, GenericSubsystem> subsystems;

  private static Indexer indexer;
  private static Launcher launcher;

  /** defines possible states of the indexer */
  public static enum IndexerState {
    IDLE,
    CHURN,
    HARD_CHURN,
    FORCE,
    FEED
  }

  /** Stores the subsystem map and retrieves the indexer instance */
  public IndexerCommands(Map<String, GenericSubsystem> systems) {
    this.subsystems = systems;
    IndexerCommands.indexer = Rebuilt.indexer;
    IndexerCommands.launcher = (Launcher) subsystems.get(Constants.LAUNCHER);
    configureTriggerStates();
  }

  public void configureButtonBindings(Controller driver, Controller operator) {
    // driver.createLeftBumper().onTrue(toggleForceFeed());
    // driver.createLeftBumper().whileTrue(shouldForceCommand()).onFalse(shouldChurnCommand());
    driver.createLeftBumper().whileTrue(shouldHardChurnCommand()).onFalse(shouldChurnCommand());
    operator
        .createLeftBumper()
        .whileTrue(
            Commands.either(
                shouldForceCommand(), shouldChurnCommand(), () -> launcher.isOKToFire()))
        .onFalse(shouldChurnCommand());
  }

  private void configureTriggerStates() {
    // Map requested states to their commands and wire triggers in a compact loop.
    // CHURN is handled separately below so it can be gated on flywheel readiness.
    java.util.Map<IndexerState, Command> stateToCommand =
        java.util.Map.of(
            IndexerState.FEED, feedStateCommand(),
            IndexerState.FORCE, forceStateCommand(),
            IndexerState.IDLE, idleStateCommand(),
            IndexerState.HARD_CHURN, hardChurnStateCommand());

    stateToCommand.forEach(
        (state, cmd) -> new Trigger(() -> indexer.isRequested(state)).onTrue(cmd));

    // CHURN: the request can be set at any time, but the indexer only physically
    // starts churning once LauncherCommands.isFlywheelReadyForChurn() is satisfied.
    new Trigger(() -> indexer.isRequested(IndexerState.CHURN)).onTrue(churnStateCommand());
  }

  public void setupDefaultCommands() {}

  /** defines command behavio for the force state stops the indexer and runs the transfer at 50% */
  public static Command forceStateCommand() {
    return Commands.runOnce(
        () -> {
          indexer.setCurrentState(IndexerState.FORCE);
          indexer.runSpindexer(Constants.Indexer.SPINDEXER_SPEED);
          indexer.runTransferFront(Constants.Indexer.TRANSFER_SPEED);
          //          indexer.runTransferBack(0.50);
        },
        indexer);
  }
  /** defines command behavior for the churn state stops the indexer and runs the transfer at 25% */
  private static Command churnStateCommand() {
    return Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.CHURN);
              indexer.runSpindexer(-0.1);
              indexer.runTransferFront(Constants.Indexer.TRANSFER_CHURN);
            },
            indexer)
        .andThen(Commands.waitSeconds(1.0))
        .andThen(
            Commands.runOnce(
                () -> {
                  indexer.runSpindexer(0.0);
                  indexer.runTransferFront(0);
                }));
  }

  private static Command hardChurnStateCommand() {
    return Commands.runOnce(
        () -> {
          indexer.setCurrentState(IndexerState.HARD_CHURN);
          indexer.runSpindexer(-0.5);
          indexer.runTransferFront(Constants.Indexer.TRANSFER_CHURN);
        },
        indexer);
  }

  /**
   * defines command behavior for the idle state stops all motors and sets the LED patters to
   * rainbow
   */
  private static Command idleStateCommand() {
    return Commands.runOnce(
        () -> {
          indexer.setCurrentState(IndexerState.IDLE);
          indexer.runSpindexer(0);
          indexer.runTransferFront(0);
          //          indexer.runTransferBack(0);
          LedStrip.changeSegmentPattern(LedStrip.ALL_LEDS, LedStrip.getRainbowPattern(0));
        },
        indexer);
  }

  // run feed command when Launcher State is idle and Operator Right Bumper is
  // pressed
  private static Command feedStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.FEED);
              indexer.runSpindexer(Constants.Indexer.SPINDEXER_SPEED);
              indexer.runTransferFront(Constants.Indexer.TRANSFER_SPEED);
              //              indexer.runTransferBack(1);
              LedStrip.changeSegmentPattern(LedStrip.ALL_LEDS, LedStrip.getRainbowPattern(25));
            },
            indexer));
  }
  /** Requests the indexer to enter the idle state */
  public static Command shouldIdleCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.IDLE));
  }
  /** Requests the indexer to enter the churn state */
  public static Command shouldChurnCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.CHURN));
  }

  public static Command churnAuto() {
    return Commands.run(
        () -> {
          indexer.runSpindexer(-0.1);
        });
  }

  public static Command shouldHardChurnCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.HARD_CHURN));
  }
  /** Requests the indexer to enter the feed state */
  public static Command shouldFeedCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.FEED));
  }

  public static Command shouldForceCommand() {
    return Commands.runOnce(
        () -> {
          indexer.setRequestedState(IndexerState.FORCE);
        });
  }

  public static Command toggleForceFeed() {
    return Commands.runOnce(
        () -> {
          if (indexer.isRequested(IndexerState.FEED)) {
            indexer.setRequestedState(IndexerState.CHURN);

          } else {
            indexer.setRequestedState(IndexerState.FEED);
          }
        });
  }
}
