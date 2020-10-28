package us.ihmc.javaSpriteWorld.examples.duncan;

class AlphaFilter
{
   private final double alpha;
   private double last = Double.NaN;

   public AlphaFilter(double alpha)
   {
      this.alpha = alpha;
   }

   public double filter(double value)
   {
      double filtered;
      if (Double.isNaN(last)) // feed forward to initial value
      {
         last = value;
         filtered = value;
      }
      else
      {
         filtered = last + (value - last) / alpha;
         last = filtered;
      }
      return filtered;
   }
}
