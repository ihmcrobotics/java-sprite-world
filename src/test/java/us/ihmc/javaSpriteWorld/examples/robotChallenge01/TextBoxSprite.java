package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.WritableImage;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCostume;

public class TextBoxSprite extends Sprite
{
   private final int fontSize;
   private final int relativeWidth;
   private final Color borderColor = Color.BLACK;
   private final Color textColor = Color.RED;

   public TextBoxSprite(String name, double height, int fontSize, int relativeWidth)
   {
      super(name);

      this.fontSize = fontSize;
      this.relativeWidth = relativeWidth;

      this.setWidth(height * relativeWidth);
      this.setHeight(height);

      this.setReflectX(false);
      this.setReflectY(true);
   }

   public void setText(String text)
   {
      SpriteCostume costume = createTextSpriteCostume(text);

      this.clearCostumes();
      this.addCostume(costume);
      this.setCostume(0);
   }

   public SpriteCostume createTextSpriteCostume(String text)
   {
      int widthPixels = fontSize * relativeWidth;
      int heightPixels = (int) (fontSize * 1.2);

      BufferedImage bufferedImage = new BufferedImage(widthPixels, heightPixels, BufferedImage.TYPE_INT_ARGB);
      Graphics2D graphics2D = bufferedImage.createGraphics();

      graphics2D.setColor(borderColor);
      graphics2D.drawRect(0, 0, widthPixels - 1, heightPixels - 1);

      graphics2D.setFont(new Font("Microsoft YaHei", Font.PLAIN, fontSize));
      graphics2D.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      graphics2D.setColor(textColor);
      graphics2D.drawString(text, fontSize / 2, (int) (fontSize));

      graphics2D.dispose();

      WritableImage fxImage = SwingFXUtils.toFXImage(bufferedImage, null);
      SpriteCostume costume = new SpriteCostume(fxImage);
      costume.setXReferencePercent(0.0);
      costume.setYReferencePercent(1.0);

      return costume;
   }

}
