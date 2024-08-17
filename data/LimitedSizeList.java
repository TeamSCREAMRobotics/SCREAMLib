package com.SCREAMLib.data;

import java.util.ArrayDeque;
import java.util.Collection;

public class LimitedSizeList<T> extends ArrayDeque<T> {
  private final int maxSize;

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
