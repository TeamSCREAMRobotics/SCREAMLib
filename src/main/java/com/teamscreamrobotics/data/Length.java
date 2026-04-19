package com.teamscreamrobotics.data;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/** An immutable, unit-safe linear distance value. Stored internally in inches. */
public class Length implements StructSerializable {

  /** A zero-length constant. */
  public static final Length kZero = new Length();

  private final double inches;

  /** Creates a zero-length value. */
  public Length() {
    this(0);
  }

  /**
   * Creates a length from a raw inch value.
   *
   * @param inches the distance in inches
   */
  public Length(double inches) {
    this.inches = inches;
  }

  /** Creates a {@link Length} from a value in inches. */
  public static Length fromInches(double inches) {
    return new Length(inches);
  }

  /** Creates a {@link Length} from a value in feet. */
  public static Length fromFeet(double feet) {
    return from(feet, 12.0);
  }

  /** Creates a {@link Length} from a value in centimeters. */
  public static Length fromCentimeters(double centimeters) {
    return from(centimeters, 1.0 / 2.54);
  }

  /** Creates a {@link Length} from a value in meters. */
  public static Length fromMeters(double meters) {
    return from(meters, 39.37);
  }

  /**
   * Converts a rotational distance to a linear distance given a wheel/spool circumference.
   *
   * @param rotations     number of full rotations
   * @param circumference the circumference per rotation
   */
  public static Length fromRotations(double rotations, Length circumference) {
    return new Length(rotations * circumference.inches);
  }

  /**
   * Creates a {@link Length} from an arbitrary unit by supplying the inches-per-unit factor.
   *
   * @param value               the measurement in the source unit
   * @param inchConversionFactor inches per source unit (e.g. {@code 39.37} for meters)
   */
  public static Length from(double value, double inchConversionFactor) {
    return new Length(value * inchConversionFactor);
  }

  /** Returns a new {@link Length} equal to {@code this + other}. */
  public Length plus(Length other) {
    return new Length(this.inches + other.getInches());
  }

  /** Returns a new {@link Length} equal to {@code this - other}. */
  public Length minus(Length other) {
    return new Length(this.inches - other.getInches());
  }

  /** Returns a new {@link Length} scaled by {@code scalar}. */
  public Length times(double scalar) {
    return new Length(this.inches * scalar);
  }

  /** Returns a new {@link Length} divided by {@code scalar}. */
  public Length div(double scalar) {
    return new Length(this.inches / scalar);
  }

  /** Returns a new {@link Length} whose inch value is the product of the two inch values. */
  public Length times(Length scalar) {
    return new Length(this.inches * scalar.inches);
  }

  /** Returns a new {@link Length} whose inch value is {@code this.inches / scalar.inches}. */
  public Length div(Length scalar) {
    return new Length(this.inches / scalar.inches);
  }

  /** Returns a new {@link Length} whose inch value is {@code inches²}. */
  public Length squared(){
    return new Length(this.inches * this.inches);
  }

  /** Returns the length in inches. */
  public double getInches() {
    return inches;
  }

  /** Returns the length in feet. */
  public double getFeet() {
    return inches / 12.0;
  }

  /** Returns the length in centimeters. */
  public double getCentimeters() {
    return inches * 2.54;
  }

  /** Returns the length in meters. */
  public double getMeters() {
    return inches / 39.37;
  }

  @Override
  public boolean equals(Object obj) {
      return this.inches == ((Length)obj).inches;
  }

  public static final LengthStruct struct = new LengthStruct();

  private static class LengthStruct implements Struct<Length> {

    @Override
    public Class<Length> getTypeClass() {
      return Length.class;
    }

    @Override
    public String getTypeName() {
      return "Length";
    }

    @Override
    public int getSize() {
      return kSizeDouble * 4;
    }

    @Override
    public String getSchema() {
      return "double inches;double feet;double centimeters;double meters";
    }

    @Override
    public Length unpack(ByteBuffer bb) {
      double inches = bb.getDouble();
      return new Length(inches);
    }

    @Override
    public void pack(ByteBuffer bb, Length value) {
      bb.putDouble(value.getInches());
      bb.putDouble(value.getFeet());
      bb.putDouble(value.getCentimeters());
      bb.putDouble(value.getMeters());
    }

    @Override
    public boolean isImmutable() {
      return true;
    }
  }
}
