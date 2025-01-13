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

  // PROPOSAL: Make this method only present in a subclass (e.g. SettableEntry)
  // PROS: Removes method that does nothing in classes that don't use it,
  //   avoiding confusion.
  // CONS: Means DataManagerEntry will need to be split into DataManagerEntry
  //   and DataManagerSettable Entry. Could be avoided by using a interface for
  //   settable. Could be avoided by removing the Entry <-> DataManagerEntry distinction.
  /** Sets the entry. {@link #update} should usually be used in place of this. */
  public void set(T newValue) {
  };
}