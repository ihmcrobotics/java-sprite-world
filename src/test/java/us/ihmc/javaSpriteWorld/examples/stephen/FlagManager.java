package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tuple2D.Point2D;

import java.util.Random;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.bodyFrameToWorldFrame;

public class FlagManager
{
   private static final Random random = new Random(3920);

   private final SLAMManager slamManager;

   private int flagIdToChase = 1;
   private boolean inDeliverFlagMode = false;
   private final Point2D[] flagLocations = new Point2D[5];
   private Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag;

   private final Point2D[] areasToExplore;
   private final Point2D areaToExplore = new Point2D();

   private int counter = 0;
   private final int counterToSwitchExploreArea = 500;
   private final double proximityToExploreAreaToSwitch = 0.4;

   public FlagManager(SLAMManager slamManager)
   {
      this.slamManager = slamManager;

      for (int i = 0; i < flagLocations.length; i++)
      {
         flagLocations[i] = new Point2D();
         flagLocations[i].setToNaN();
      }

      areasToExplore = new Point2D[9];
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            areasToExplore[3 * i + j] = new Point2D(5 + 3 * (i - 1), 5 + 3 * (j - 1));
         }
      }

      switchAreaToExplore();
   }

   public void update()
   {
      if (counter++ >= counterToSwitchExploreArea)
      {
         switchAreaToExplore();
         counter = 0;
      }
      else if (areaToExplore.distance(slamManager.getXYPosition()) < proximityToExploreAreaToSwitch)
      {
         switchAreaToExplore();
         counter = 0;
      }

      takeANoteOfFlagLocation();
   }

   private void switchAreaToExplore()
   {
      while (true)
      {
         Point2D newAreaToExplore = areasToExplore[random.nextInt(areasToExplore.length)];
         if (newAreaToExplore.equals(areaToExplore))
         {
            continue;
         }
         else
         {
            areaToExplore.set(newAreaToExplore);
            return;
         }
      }
   }

   private void takeANoteOfFlagLocation()
   {
      int flagZeroIndexId = positionInBodyFrameAndIdOfClosestFlag.getRight() - 1;
      bodyFrameToWorldFrame(positionInBodyFrameAndIdOfClosestFlag.getLeft(), flagLocations[flagZeroIndexId], slamManager.getHeading(), slamManager.getXYPosition());
   }

   public boolean isInDeliverFlagMode()
   {
      return inDeliverFlagMode;
   }

   public int getFlagIdToChase()
   {
      return flagIdToChase;
   }

   public boolean hasDetectedNextFlag()
   {
      return !getNextFlagLocation().containsNaN();
   }

   public Point2D getAreaToExplore()
   {
      return areaToExplore;
   }

   public Point2D getNextFlagLocation()
   {
      return flagLocations[flagIdToChase - 1];
   }

   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      this.positionInBodyFrameAndIdOfClosestFlag = positionInBodyFrameAndIdOfClosestFlag;
   }

   public boolean getDropFlag()
   {
      return inDeliverFlagMode && (slamManager.getXYPosition().getX() > 8.15 && slamManager.getXYPosition().getY() > 8.15);
   }

   public void sensePickedUpFlag(int id)
   {
      if (id == flagIdToChase)
      {
         inDeliverFlagMode = true;
      }
      else if (inDeliverFlagMode)
      {
         inDeliverFlagMode = false;
      }

      flagLocations[id - 1].setToNaN();

      System.out.println("pickedUpFlag " + id);
   }

   private int numberOfRoundsCompleted = 0;

   public void senseDeliveredFlag(int flagId)
   {
      System.out.println("deliveredFlag " + flagId);

      if (inDeliverFlagMode)
      {
         if (flagIdToChase == 5)
         {
            flagIdToChase = 1;

            numberOfRoundsCompleted++;
            System.out.println("====================================================");
            System.out.println("             FINISHED " + numberOfRoundsCompleted + " TIMES");
            System.out.println("====================================================");
         }
         else
         {
            flagIdToChase++;
         }

         inDeliverFlagMode = false;
      }
      else
      {
         throw new RuntimeException("Shouldn't get here...");
      }
   }
}
