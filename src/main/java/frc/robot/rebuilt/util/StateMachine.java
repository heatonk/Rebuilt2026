// Original source: https://github.com/tom131313/AdvancedCommanddExamples/tree/main
package frc.robot.rebuilt.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class StateMachine extends Command {

  private final String name;
  private boolean exitStateMachine;
  private final EventLoop events = new EventLoop();
  private final List<WeakReference<State>> states = new ArrayList<>();
  private State initialState = null;
  private State completedNormally = null;
  private Command stateCommandAugmentedPrevious = null;

  public StateMachine(String name) {
    requireNonNullParam(name, "name", "StateMachine");
    this.name = name;
  }

  public void setInitialState(State initialState) {
    requireNonNullParam(initialState, "initialState", "StateMachine.setInitialState");
    this.initialState = initialState;
  }

  public State addState(String name, Command stateCommand) {
    return new State(name, stateCommand);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("All states for StateMachine ").append(name).append("\n");

    for (WeakReference<State> state : states) {
      boolean noExits = true;
      boolean noEntrances = true;

      sb.append("-------")
          .append(state.get().name)
          .append("-------")
          .append(state.get() == initialState ? " INITIAL STATE\n" : "\n");

      for (Transition transition : state.get().transitions) {
        noExits = false;
        sb.append("transition ")
            .append(transition)
            .append(" to ")
            .append(transition.nextState != null ? transition.nextState.name : "exit StateMachine")
            .append(" onTrue trigger ")
            .append(transition.triggeringEvent)
            .append("\n");
      }

      allStates:
      for (WeakReference<State> stateInner : states) {
        for (Transition transition : stateInner.get().transitions) {
          if (transition.nextState == state.get()) {
            noEntrances = false;
            break allStates;
          }
        }
      }
      sb.append(
          (noEntrances
              ? "Caution - State has no entrances and will not be used.\n\n"
              : noExits
                  ? "Notice - State has no exits and if entered will either stop or hang the StateMachine command.\n\n"
                  : "\n"));
    }
    return sb.toString();
  }

  @Override
  public void initialize() {
    exitStateMachine = false;
    events.clear();
    CommandScheduler.getInstance()
        .schedule(new ScheduleCommand(initialState.stateCommandAugmented));
  }

  @Override
  public void execute() {
    events.poll();
  }

  @Override
  public void end(boolean interrupted) {
    if (stateCommandAugmentedPrevious != null) {
      stateCommandAugmentedPrevious.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return exitStateMachine;
  }

  private class WrapState extends WrapperCommand {
    State state;

    WrapState(State state, Command command) {
      super(command);
      this.state = state;
    }

    @Override
    public void initialize() {
      events.clear();
      if (stateCommandAugmentedPrevious != null) {
        stateCommandAugmentedPrevious.cancel();
      }
      if (!state.transitions.isEmpty()) {
        for (Transition transition : state.transitions) {
          var trigger = new Trigger(events, transition.triggeringEvent);
          if (transition.nextState == null) {
            trigger.onTrue(Commands.runOnce(() -> exitStateMachine = true).ignoringDisable(true));
          } else {
            trigger.onTrue(transition.nextState.stateCommandAugmented);
          }
        }
      }

      completedNormally = null;
      stateCommandAugmentedPrevious = this;

      m_command.initialize();
    }

    @Override
    public void end(boolean interrupted) {
      m_command.end(interrupted);
      stateCommandAugmentedPrevious = null;

      if (state.transitions.isEmpty()) {
        exitStateMachine = true;
      } else {
        if (!interrupted) {
          completedNormally = state;
          for (Transition transition : state.transitions) {
            if (transition.triggeringEvent == state.whenCompleteCondition) {
              if (transition.nextState == null) {
                exitStateMachine = true;
              }
              break;
            }
          }
        }
      }
    }
  }

  public class State extends Command {
    private final String name;
    private Command stateCommandAugmented;
    private List<Transition> transitions = new ArrayList<>();
    private BooleanSupplier whenCompleteCondition = () -> State.this == completedNormally;

    private State(String name, Command stateCommand) {
      this.name = name;
      StateMachine.this.states.add(new WeakReference<>(this));
      this.stateCommandAugmented = new WrapState(this, stateCommand);
    }

    public TransitionNeedsConditionStage switchTo(State to) {
      requireNonNullParam(to, "to", "State.switchTo");
      return new TransitionNeedsTargetStage().to(to);
    }

    public TransitionNeedsConditionStage exitStateMachine() {
      return new TransitionNeedsConditionStage(null);
    }

    public final class TransitionNeedsTargetStage {
      private TransitionNeedsConditionStage to(State to) {
        return new TransitionNeedsConditionStage(to);
      }
    }

    public final class TransitionNeedsConditionStage {
      private final State m_targetState;

      private TransitionNeedsConditionStage(State to) {
        m_targetState = to;
      }

      public void when(BooleanSupplier condition) {
        checkDuplicateCondition(condition);
        transitions.add(new Transition(m_targetState, condition));
      }

      public void whenComplete() {
        checkDuplicateCondition(whenCompleteCondition);
        transitions.add(new Transition(m_targetState, whenCompleteCondition));
      }

      private void checkDuplicateCondition(BooleanSupplier condition) {
        for (Transition transition : transitions) {
          if (transition.triggeringEvent == condition) {
            throw new IllegalArgumentException("Condition object can be used only once per state.");
          }
        }
      }
    }
  }

  private class Transition {
    State nextState;
    BooleanSupplier triggeringEvent;

    private Transition(State toNextState, BooleanSupplier whenEvent) {
      this.nextState = toNextState;
      this.triggeringEvent = whenEvent;
    }
  }
}
