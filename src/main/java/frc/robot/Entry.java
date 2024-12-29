package frc.robot;

import frc.robot.DataManager.DataManagerEntry;

/**
 * A data entry. If the entry is for use with {@link DataManager},
 * use a {@link DataManagerEntry} instead; it ensures the entry is automatically
 * connected to {@link DataManager#update DataManager.instance().update}.
 */
public abstract class Entry<T> {
  public Entry() {
  }

  /** Updates the entry, called in DataManager.update() */
  public void update() {
  };

  /** Gets the entry. Called from DataManager.instance.entry.get() */
  public abstract T get();

  /** Sets the entry. {@link #update} should usually be used in place of this. */
  public void set(T newValue) {
  };
}