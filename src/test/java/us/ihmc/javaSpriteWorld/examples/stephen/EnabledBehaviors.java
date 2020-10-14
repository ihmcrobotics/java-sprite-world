package us.ihmc.javaSpriteWorld.examples.stephen;

public class EnabledBehaviors
{
   private boolean wallEnabled;
   private boolean foodEnabled;
   private boolean predatorEnabled;
   private boolean flagEnabled;

   public boolean isWallEnabled()
   {
      return wallEnabled;
   }

   public void setWallEnabled(boolean wallEnabled)
   {
      this.wallEnabled = wallEnabled;
   }

   public boolean isFoodEnabled()
   {
      return foodEnabled;
   }

   public void setFoodEnabled(boolean foodEnabled)
   {
      this.foodEnabled = foodEnabled;
   }

   public boolean isPredatorEnabled()
   {
      return predatorEnabled;
   }

   public void setPredatorEnabled(boolean predatorEnabled)
   {
      this.predatorEnabled = predatorEnabled;
   }

   public boolean isFlagEnabled()
   {
      return flagEnabled;
   }

   public void setFlagEnabled(boolean flagEnabled)
   {
      this.flagEnabled = flagEnabled;
   }
}
