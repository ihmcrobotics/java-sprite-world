package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

public class BehaviorStatusHolder
{
   private double wallWeight;
   private double predatorWeight;
   private double hungerWeight;
   private boolean hasFlag;

   private final double[] wallAction = new double[2];
   private final double[] predatorAction = new double[2];
   private final double[] hungerAction = new double[2];
   private final double[] flagAction = new double[2];
   private boolean dropFlag;
   private long trappedTime;

   public double getWallWeight()
   {
      return wallWeight;
   }

   public double getPredatorWeight()
   {
      return predatorWeight;
   }

   public double getHungerWeight()
   {
      return hungerWeight;
   }

   public boolean isHasFlag()
   {
      return hasFlag;
   }

   public double[] getWallAction()
   {
      return wallAction;
   }

   public double[] getPredatorAction()
   {
      return predatorAction;
   }

   public double[] getHungerAction()
   {
      return hungerAction;
   }

   public double[] getFlagAction()
   {
      return flagAction;
   }

   public boolean isDropFlag()
   {
      return dropFlag;
   }

   public void setWallWeight(double wallWeight)
   {
      this.wallWeight = wallWeight;
   }

   public void setPredatorWeight(double predatorWeight)
   {
      this.predatorWeight = predatorWeight;
   }

   public void setHungerWeight(double hungerWeight)
   {
      this.hungerWeight = hungerWeight;
   }

   public void setHasFlag(boolean hasFlag)
   {
      this.hasFlag = hasFlag;
   }

   public void wallAction(double acceleration, double turnRate)
   {
      wallAction[0] = acceleration;
      wallAction[1] = turnRate;
   }

   public void predatorAction(double acceleration, double turnRate)
   {
      predatorAction[0] = acceleration;
      predatorAction[1] = turnRate;
   }

   public void hungerAction(double acceleration, double turnRate)
   {
      hungerAction[0] = acceleration;
      hungerAction[1] = turnRate;
   }

   public void flagAction(double acceleration, double turnRate)
   {
      flagAction[0] = acceleration;
      flagAction[1] = turnRate;
   }

   public void setDropFlag(boolean dropFlag)
   {
      this.dropFlag = dropFlag;
   }

   public void setTrapped(long trappedTime)
   {
      this.trappedTime = trappedTime;
   }

   public long getTrappedTime()
   {
      return trappedTime;
   }
}
