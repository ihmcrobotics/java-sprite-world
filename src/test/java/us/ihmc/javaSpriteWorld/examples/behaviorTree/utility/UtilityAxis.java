package us.ihmc.javaSpriteWorld.examples.behaviorTree.utility;

import us.ihmc.commons.MathTools;

import java.util.function.DoubleSupplier;

public abstract class UtilityAxis
{
   protected final double m;
   protected final double k;
   protected final double b;
   protected final double c;
   private final DoubleSupplier xSupplier;

   public UtilityAxis(double m, double k, double b, double c, DoubleSupplier xSupplier)
   {
      this.m = m;
      this.k = k;
      this.b = b;
      this.c = c;
      this.xSupplier = xSupplier;
   }

   public double calculate()
   {
      return MathTools.clamp(calculateInternal(xSupplier.getAsDouble()), 0.0, 1.0);
   }

   protected abstract double calculateInternal(double x);
}
