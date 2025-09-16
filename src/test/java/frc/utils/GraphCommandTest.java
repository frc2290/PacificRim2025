package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class GraphCommandTest {

  @Test
  void transitionsRunTargetAndArrivalCommands() {
    GraphCommand command = new GraphCommand();

    AtomicInteger targetRuns = new AtomicInteger();
    AtomicInteger arrivalRuns = new AtomicInteger();

    Command noopCommand = mock(Command.class);

    AtomicBoolean targetScheduled = new AtomicBoolean(false);
    Command targetCommand = mock(Command.class);
    doAnswer(
            invocation -> {
              targetScheduled.set(true);
              targetRuns.incrementAndGet();
              targetScheduled.set(false);
              return null;
            })
        .when(targetCommand)
        .schedule();
    when(targetCommand.isScheduled()).thenAnswer(invocation -> targetScheduled.get());
    doAnswer(
            invocation -> {
              targetScheduled.set(false);
              return null;
            })
        .when(targetCommand)
        .cancel();

    Command arrivalCommand = mock(Command.class);
    doAnswer(
            invocation -> {
              arrivalRuns.incrementAndGet();
              return null;
            })
        .when(arrivalCommand)
        .schedule();

    GraphCommand.GraphCommandNode start =
        command.new GraphCommandNode("Start", noopCommand, noopCommand, noopCommand);
    GraphCommand.GraphCommandNode target =
        command.new GraphCommandNode("Target", targetCommand, noopCommand, arrivalCommand);

    start.AddNode(target, 1.0);

    command.setGraphRootNode(start);
    command.initialize();

    command.setTargetNode(target);
    command.execute();

    assertTrue(command.isTransitioning());
    assertSame(target, command.getCurrentNode());

    command.execute();
    assertFalse(command.isTransitioning());
    command.execute();

    assertSame(target, command.getCurrentNode());
    assertFalse(command.isTransitioning());
    assertEquals(1, targetRuns.get());
    assertEquals(1, arrivalRuns.get());
  }

  @Test
  void selectsLowestCostWaypoint() {
    GraphCommand command = new GraphCommand();

    GraphCommand.GraphCommandNode start = createNode(command, "Start");
    GraphCommand.GraphCommandNode slow = createNode(command, "Slow");
    GraphCommand.GraphCommandNode fast = createNode(command, "Fast");
    GraphCommand.GraphCommandNode target = createNode(command, "Target");

    start.AddNode(slow, 5.0);
    slow.AddNode(target, 5.0);
    start.AddNode(fast, 1.0);
    fast.AddNode(target, 1.0);

    command.setGraphRootNode(start);
    command.initialize();

    assertSame(fast, start.getNextNodeGivenTarget(target));
    assertEquals(2.0, start.getCost(target), 1e-9);
  }

  private GraphCommand.GraphCommandNode createNode(GraphCommand command, String name) {
    return command
    .new GraphCommandNode(
        name,
        new InstantCommand(() -> {}),
        new InstantCommand(() -> {}),
        new InstantCommand(() -> {}));
  }
}
