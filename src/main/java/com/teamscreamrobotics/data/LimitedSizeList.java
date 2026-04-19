package com.teamscreamrobotics.data;

import java.util.ArrayDeque;
import java.util.Collection;

/**
 * A fixed-capacity deque that automatically evicts the oldest element when full.
 * Useful for rolling windows of sensor readings or log history.
 *
 * @param <T> the type of elements held in this list
 */
public class LimitedSizeList<T> extends ArrayDeque<T> {
  private final int maxSize;

  /**
   * Creates a new limited-size list.
   *
   * @param maxSize the maximum number of elements to retain
   */
  public LimitedSizeList(int maxSize) {
    super(maxSize);
    this.maxSize = maxSize;
  }

  @Override
  public boolean add(T element) {
    if (this.size() >= maxSize) {
      this.removeFirst();
    }
    return super.add(element);
  }

  @Override
  public boolean addAll(Collection<? extends T> c) {
    for (T element : c) {
      add(element);
    }
    return true;
  }
}
