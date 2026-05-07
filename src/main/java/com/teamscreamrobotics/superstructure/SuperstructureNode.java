package com.teamscreamrobotics.superstructure;

import java.util.Objects;

/** A named node in the collision-avoidance graph representing a known-safe mechanism configuration. */
public class SuperstructureNode {

    public final String name;
    public final SuperstructurePosition position;

    public SuperstructureNode(String name, SuperstructurePosition position) {
        this.name = name;
        this.position = position;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof SuperstructureNode other)) return false;
        return Objects.equals(name, other.name);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(name);
    }

    @Override
    public String toString() {
        return "SuperstructureNode(" + name + ")";
    }
}
