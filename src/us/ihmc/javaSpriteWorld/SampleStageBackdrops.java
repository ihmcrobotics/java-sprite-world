package us.ihmc.javaSpriteWorld;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URL;

import javax.imageio.ImageIO;

public class SampleStageBackdrops
{

   public static StageBackdrop getBackgammonBoard()
   {
      return StageBackdrop.createFromFile("SampleImages/BackgammonBoard.jpg");
   }
   
   public static StageBackdrop getWhiteChessKing()
   {
      return StageBackdrop.createFromFile("SampleImages/portablejim-Chess-tile-King-300px.png");
   }
   
   public static StageBackdrop getCrossHairs()
   {
	   StageBackdrop crossHairs = StageBackdrop.createFromFile("SampleImages/Crosshairs-3456-300px.png");
	   crossHairs.setXReferencePercent(0.50667);
	   crossHairs.setYReferencePercent(0.50667);
	   return crossHairs;
   }
   

}
