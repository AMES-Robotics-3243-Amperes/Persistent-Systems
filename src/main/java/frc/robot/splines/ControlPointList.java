package frc.robot.splines;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.ListIterator;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.splines.tasks.Task;

/**
 * Working with {@link Task Tasks} and {@link Translation2d Translation2ds} at
 * the same time in the way that that {@link Path} does is a little bit ugly and
 * requires two lists to be used together in a way that can potentially go awry.
 * For the purposes of ease of use and safety, this class can independently
 * manage many {@link Translation2d Translation2ds} and potential corresponding
 * {@link Task Tasks}. It ensures that calls to {@link #getTranslations} don't
 * return consecutive, identical pairs of {@link Translation2d Translation2ds}
 * and provides {@link #getTasksWithLength} to determine the length across a
 * spline that {@link Task Tasks} occur at.
 */
public class ControlPointList {
  /**
   * Contains a {@link Task} and the index of the unique {@link Translation2d} at
   * which it occurs.
   */
  private class TaskWithTranslationIndex {
    public Task task;
    public int index;

    public TaskWithTranslationIndex(Task task, int index) {
      this.task = task;
      this.index = index;
    }
  }

  private ArrayList<TaskWithTranslationIndex> tasksWithIndices = new ArrayList<TaskWithTranslationIndex>();
  private ArrayList<Translation2d> uniqueTranslations = new ArrayList<Translation2d>();

  public void addControlPoint(Translation2d translation, Optional<Task> task) {
    if (uniqueTranslations.isEmpty()) {
      uniqueTranslations.add(translation);
    } else if (uniqueTranslations.get(uniqueTranslations.size() - 1) != translation) {
      uniqueTranslations.add(translation);
    }

    if (task.isPresent()) {
      tasksWithIndices.add(new TaskWithTranslationIndex(task.get(), uniqueTranslations.size() - 1));
    }
  }

  public void addControlPoint(Translation2d translation, Task task) {
    addControlPoint(translation, Optional.ofNullable(task));
  }

  public void addControlPoint(Translation2d translation) {
    addControlPoint(translation, Optional.empty());
  }

  public List<Translation2d> getTranslations() {
    return uniqueTranslations;
  }

  public List<Task> getTasks() {
    return tasksWithIndices.stream().map(taskWithIndex -> taskWithIndex.task).toList();
  }

  public List<Task> getUpcomingTasks(double currentLength) {
    return getTasks().stream().filter(task -> task.isUpcoming(currentLength)).toList();
  }

  public List<Task> getActiveTasks(double currentLength) {
    return getTasks().stream().filter(task -> task.isActive(currentLength)).toList();
  }

  /**
   * Similar go {@link #getActiveTasks}, but if some subsystem has multiple active
   * tasks, only includes the first.
   * 
   * @return The next active tasks
   */
  public List<Task> getValidActiveTasks(double length) {
    List<Task> activeTasks = getActiveTasks(length);
    List<Task> validActiveTasks = new ArrayList<Task>();
    HashSet<Subsystem> activeSubsystems = new HashSet<Subsystem>();
    for (Task task : activeTasks) {
      if (task.getRequirements().stream().anyMatch(subsystem -> activeSubsystems.contains(subsystem))) {
        continue;
      }

      activeSubsystems.addAll(task.getRequirements());
      validActiveTasks.add(task);
    }

    return validActiveTasks;
  }

  public void initializeTasks(Spline spline, boolean splineInterpolatedFromStart) {
    Function<TaskWithTranslationIndex, Double> calculateTargetLength = splineInterpolatedFromStart
        ? taskWithIndex -> spline.arcLength((taskWithIndex.index + 1.0) / (double) uniqueTranslations.size())
        : taskWithIndex -> spline.arcLength((double) taskWithIndex.index / (uniqueTranslations.size() - 1.0));

    for (TaskWithTranslationIndex taskWithIndex : tasksWithIndices) {
      taskWithIndex.task.initialize(spline, calculateTargetLength.apply(taskWithIndex));
    }

    // make sure that no task ends after a task that proceeds it. this allows us to
    // only deal with one task per subsystem at a time
    double earliestEnd = Double.MAX_VALUE;
    List<Task> tasks = getTasks();
    ListIterator<Task> it = tasks.listIterator(tasks.size());
    while (it.hasPrevious()) {
      Task nextTask = it.previous();
      earliestEnd = Double.min(nextTask.getEndLength(), earliestEnd);

      nextTask.setEndLength(earliestEnd);
    }
  }
}
