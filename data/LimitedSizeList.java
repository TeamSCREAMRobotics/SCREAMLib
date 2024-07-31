package com.SCREAMLib.data;

import java.util.ArrayList;
import java.util.LinkedList;

public class LimitedSizeList<T> extends LinkedList<T> {
    private final int maxSize;

    public LimitedSizeList(int maxSize) {
        this.maxSize = maxSize - 1;
    }

    @Override
    public boolean add(T element) {
        if (this.size() > maxSize) {
            this.removeFirst();
        }
        return super.add(element);
    }
}
