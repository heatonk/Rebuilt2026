// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.util;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks every controller button binding registered during {@code configureButtonBindings} and
 * fails fast if the same {@code (controller, input, edge)} triple is bound twice within the active
 * scope.
 *
 * <p>The {@link Controller} factory methods (e.g. {@code createAButton}) return tracked button
 * subclasses that call {@link #register} from their {@code onTrue}/{@code onFalse}/etc. overrides.
 * Sensor-driven {@code new Trigger(() -> ...)} bindings and axis post-processing are intentionally
 * not tracked — only true controller-input bindings flow through this registry.
 *
 * <p>Scopes exist because teleop and test mode each call their own {@code configureButtonBindings}
 * variant. A scope is opened with {@link #beginScope} and closed with {@link #assertAndPublish},
 * which asserts no duplicates and emits an AdvantageKit log snapshot of the bindings in that scope.
 */
public final class ButtonBindingRegistry {

  /** Which Trigger edge a binding fires on. */
  public enum Edge {
    ON_TRUE,
    ON_FALSE,
    WHILE_TRUE,
    WHILE_FALSE,
    TOGGLE_ON_TRUE,
    TOGGLE_ON_FALSE
  }

  /** A single registered binding. */
  public record Binding(
      String scope,
      String controllerName,
      String inputName,
      Edge edge,
      String commandName,
      String ownerClass) {}

  private static final Map<String, List<Binding>> SCOPES = new LinkedHashMap<>();
  private static String activeScope = null;

  private ButtonBindingRegistry() {}

  /**
   * Set the active scope. New registrations land here. If the scope hasn't been seen before it is
   * created empty; otherwise existing bindings are preserved so callers can split registration
   * across multiple methods.
   */
  public static synchronized void beginScope(String scope) {
    activeScope = scope;
    SCOPES.computeIfAbsent(scope, k -> new ArrayList<>());
  }

  /** Drop all bindings recorded for {@code scope}. Useful if a configure method is re-run. */
  public static synchronized void clearScope(String scope) {
    SCOPES.remove(scope);
    if (scope.equals(activeScope)) {
      activeScope = null;
    }
  }

  /**
   * Record a binding under the currently active scope. Intended to be called only from the tracked
   * button subclasses in {@link Controller}.
   */
  public static synchronized void register(
      String controllerName, String inputName, Edge edge, Command command) {
    if (activeScope == null) {
      // No scope was opened — record under "default" so we still capture something useful.
      beginScope("default");
    }
    String commandName = command == null ? "<null>" : command.getName();
    SCOPES
        .get(activeScope)
        .add(new Binding(activeScope, controllerName, inputName, edge, commandName, findOwner()));
  }

  /**
   * Validate the named scope and publish a snapshot to AdvantageKit. Throws if any {@code
   * (controller, input, edge)} triple appears more than once in this scope.
   */
  public static synchronized void assertAndPublish(String scope) {
    List<Binding> bindings = SCOPES.getOrDefault(scope, List.of());

    Map<String, List<Binding>> byKey = new LinkedHashMap<>();
    for (Binding b : bindings) {
      String key = b.controllerName() + "." + b.inputName() + "." + b.edge();
      byKey.computeIfAbsent(key, k -> new ArrayList<>()).add(b);
    }

    List<String> conflicts = new ArrayList<>();
    for (Map.Entry<String, List<Binding>> e : byKey.entrySet()) {
      if (e.getValue().size() > 1) {
        StringBuilder sb = new StringBuilder();
        sb.append(e.getKey()).append(" bound ").append(e.getValue().size()).append(" times: ");
        for (int i = 0; i < e.getValue().size(); i++) {
          Binding b = e.getValue().get(i);
          if (i > 0) sb.append(", ");
          sb.append(b.ownerClass()).append(" -> ").append(b.commandName());
        }
        conflicts.add(sb.toString());
      }
    }

    publishToAkit(scope, bindings);

    if (!conflicts.isEmpty()) {
      StringBuilder msg = new StringBuilder();
      msg.append("ButtonBindingRegistry: ")
          .append(conflicts.size())
          .append(" duplicate binding(s) in scope '")
          .append(scope)
          .append("':\n");
      for (String c : conflicts) {
        msg.append("  - ").append(c).append('\n');
      }
      throw new IllegalStateException(msg.toString());
    }
  }

  /** Returns an unmodifiable view of the bindings in a scope. Intended for tests / debugging. */
  public static synchronized List<Binding> getBindings(String scope) {
    return List.copyOf(SCOPES.getOrDefault(scope, List.of()));
  }

  private static void publishToAkit(String scope, List<Binding> bindings) {
    String prefix = "ButtonBindings/" + scope + "/";
    Map<String, Integer> seen = new HashMap<>();
    for (Binding b : bindings) {
      String base = prefix + b.controllerName() + "/" + b.inputName() + "_" + b.edge();
      int n = seen.merge(base, 1, Integer::sum);
      String key = n == 1 ? base : base + "#" + n;
      Logger.recordOutput(key, b.ownerClass() + " -> " + b.commandName());
    }
    Logger.recordOutput(prefix + "_count", bindings.size());
  }

  // Walk the stack to find the first frame that lives in frc.robot.rebuilt.* but is not the
  // Controller, the tracked button classes, or this registry. That's the call site that wrote
  // the binding — useful when reporting conflicts.
  private static String findOwner() {
    StackTraceElement[] stack = Thread.currentThread().getStackTrace();
    for (StackTraceElement frame : stack) {
      String cls = frame.getClassName();
      if (!cls.startsWith("frc.robot.")) continue;
      if (cls.equals(ButtonBindingRegistry.class.getName())) continue;
      if (cls.startsWith(Controller.class.getName())) continue;
      return cls + "." + frame.getMethodName() + ":" + frame.getLineNumber();
    }
    return "<unknown>";
  }
}
