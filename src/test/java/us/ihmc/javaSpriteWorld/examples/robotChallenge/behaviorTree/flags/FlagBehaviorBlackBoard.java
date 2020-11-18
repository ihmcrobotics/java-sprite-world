package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.flags;

import us.ihmc.euclid.tuple2D.Point2D;

public class FlagBehaviorBlackBoard
{
   private int flagIdToChase;
   private boolean hasCorrectFlag;
   private final Point2D[] flagLocations = new Point2D[5];

   public FlagBehaviorBlackBoard()
   {
      for (int i = 0; i < flagLocations.length; i++)
      {
         flagLocations[i] = new Point2D();
      }

      reset();
   }

   public void reset()
   {
      for (int i = 0; i < flagLocations.length; i++)
      {
         flagLocations[i].setToNaN();
      }

      flagIdToChase = 1;
      hasCorrectFlag = false;
   }

   public int getFlagIdToChase()
   {
      return flagIdToChase;
   }

   public void setHasCorrectFlag(boolean hasCorrectFlag)
   {
      this.hasCorrectFlag = hasCorrectFlag;
   }

   public boolean hasCorrectFlag()
   {
      return hasCorrectFlag;
   }

   public void deliveredFlag()
   {
      if (flagIdToChase == 5)
         flagIdToChase = 1;
      else
         flagIdToChase++;

      hasCorrectFlag = false;
   }

   public Point2D getNextFlagLocation()
   {
      return getFlagLocation(flagIdToChase);
   }

   public Point2D getFlagLocation(int flagNumber)
   {
      return flagLocations[flagNumber - 1];
   }

   public boolean hasSeenNextFlag()
   {
      return !flagLocations[flagIdToChase - 1].containsNaN();
   }
}
