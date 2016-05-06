package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;

public class SpriteStage
{
   private ArrayList<StageBackdrop> backdrops = new ArrayList<>();
   private int backdropNumber = -1;
   private final String name;
   
   public SpriteStage(String name)
   {
      this.name = name;
   }
     
   public int addBackdrop(String filename)
   {
	   StageBackdrop stageBackdrop = StageBackdrop.createFromFile(filename);
	   return this.addBackdrop(stageBackdrop);
   }
   
   public int addBackdrop(StageBackdrop stageBackdrop)
   {
      backdrops.add(stageBackdrop);
      int backdropNumber = backdrops.size() - 1;
      if (backdropNumber == 0) this.backdropNumber = 0;

      return backdropNumber;
   }
   
   public void switchToBackDrop(int backDropNumber)
   {
      this.backdropNumber = backDropNumber;
   }

   public StageBackdrop getBackdrop()
   {
      if ((backdropNumber >= 0) && (backdropNumber < backdrops.size()))
         return backdrops.get(backdropNumber);
      
      else return null;
   }
}
