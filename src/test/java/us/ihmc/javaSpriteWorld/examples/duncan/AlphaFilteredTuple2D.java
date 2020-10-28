package us.ihmc.javaSpriteWorld.examples.duncan;

import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;

class AlphaFilteredTuple2D
{
   protected final Tuple2DBasics tuple;
   protected final AlphaFilter xFilter;
   protected final AlphaFilter yFilter;

   public AlphaFilteredTuple2D(Tuple2DBasics tuple, double alpha)
   {
      this.tuple = tuple;
      this.xFilter = new AlphaFilter(alpha);
      this.yFilter = new AlphaFilter(alpha);
   }

   public void filter()
   {
      tuple.set(xFilter.filter(tuple.getX()), yFilter.filter(tuple.getY()));
   }
}
