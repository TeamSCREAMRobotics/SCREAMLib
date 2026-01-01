package data;

public class DataHelpers {
  @FunctionalInterface
  public static interface TriConsumer<T1, T2, T3> {
    void accept(T1 t1, T2 t2, T3 t3);
  }

  public static record Dimensions(double height, double width) {}

  public static record LoggedOutput<T>(String key, T value){}
}
