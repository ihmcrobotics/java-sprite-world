package us.ihmc.javaSpriteWorld;

import javafx.scene.image.Image;

public class StageBackdrop extends SpriteCostume
{
   public static StageBackdrop createFromFile(String filename)
   {
      return new StageBackdrop(SpriteCostume.createFromFile(filename));
   }

   public StageBackdrop(Image image)
   {
      super(image);
   }

   public StageBackdrop(SpriteCostume costume)
   {
      super(costume);
   }

}
