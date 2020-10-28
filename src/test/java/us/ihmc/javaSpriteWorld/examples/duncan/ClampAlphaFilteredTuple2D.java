package us.ihmc.javaSpriteWorld.examples.duncan;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;

class ClampAlphaFilteredTuple2D extends AlphaFilteredTuple2D
{
   private double clamp;

   public ClampAlphaFilteredTuple2D(Tuple2DBasics tuple, double alpha, double clamp)
   {
      super(tuple, alpha);
      this.clamp = clamp;
   }

   @Override
   public void filter()
   {
      tuple.set(xFilter.filter(MathTools.clamp(tuple.getX(), clamp)), yFilter.filter(MathTools.clamp(tuple.getY(), clamp)));
   }
}
