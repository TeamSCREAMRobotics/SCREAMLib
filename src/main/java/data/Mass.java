package data;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class Mass implements StructSerializable {

  public static final Mass kZero = new Mass();

  private final double pounds;

  public Mass() {
    this(0);
  }

  public Mass(double pounds) {
    this.pounds = pounds;
  }

  public static Mass fromPounds(double pounds) {
    return new Mass(pounds);
  }

  public static Mass fromKilograms(double kilograms) {
    return fromPounds(Units.kilogramsToLbs(kilograms));
  }

  public static Mass from(double value, double poundConversionFactor) {
    return new Mass(value * poundConversionFactor);
  }

  public Mass plus(Length other) {
    return new Mass(this.pounds + other.getInches());
  }

  public Mass minus(Length other) {
    return new Mass(this.pounds - other.getInches());
  }

  public Mass times(double scalar) {
    return new Mass(this.pounds * scalar);
  }

  public Mass div(double scalar) {
    return new Mass(this.pounds / scalar);
  }

  public Mass times(Mass scalar) {
    return new Mass(this.pounds * scalar.pounds);
  }

  public Mass div(Mass scalar) {
    return new Mass(this.pounds / scalar.pounds);
  }

  public Mass squared() {
    return new Mass(this.pounds * this.pounds);
  }

  public double getPounds() {
    return pounds;
  }

  public double getKilograms() {
    return Units.lbsToKilograms(pounds);
  }

  @Override
  public boolean equals(Object obj) {
    return this.pounds == ((Mass) obj).pounds;
  }

  public static final MassStruct struct = new MassStruct();

  private static class MassStruct implements Struct<Mass> {

    @Override
    public Class<Mass> getTypeClass() {
      return Mass.class;
    }

    @Override
    public String getTypeName() {
      return "Mass";
    }

    @Override
    public int getSize() {
      return kSizeDouble * 2;
    }

    @Override
    public String getSchema() {
      return "double pounds;double kilograms";
    }

    @Override
    public Mass unpack(ByteBuffer bb) {
      double pounds = bb.getDouble();
      return new Mass(pounds);
    }

    @Override
    public void pack(ByteBuffer bb, Mass value) {
      bb.putDouble(value.getPounds());
      bb.putDouble(value.getKilograms());
    }

    @Override
    public boolean isImmutable() {
      return true;
    }
  }
}
