package data;

public class DataHelpers {
  @FunctionalInterface
  public interface TriConsumer<T1, T2, T3> {
    void accept(T1 t1, T2 t2, T3 t3);
  }

  public record Dimensions(double height, double width) {}

  public record LoggedOutput<T>(String key, T value){}
}
