package us.ihmc.javaSpriteWorld;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Window;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Gdx2DPixmap;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import javafx.scene.control.Button;
import javafx.scene.image.PixelReader;
import javafx.scene.image.WritablePixelFormat;
import us.ihmc.javaSpriteWorld.libgdx.LibGDXLwjgl3ApplicationAdapter;

import javax.swing.*;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.HashMap;

import static com.badlogic.gdx.graphics.g2d.Gdx2DPixmap.GDX2D_FORMAT_RGBA8888;

public class SpriteWorldViewerUsingLibGDX implements SpriteWorldViewer
{
   private final String name;
   private int preferredWidth = 1000;
   private int preferredHeight = 1000;
   private boolean resizable = false;
   private SpriteWorld spriteWorld;
   private int startupLocationOnScreenX = -1;
   private int startupLocationOnScreenY = -1;
   private Lwjgl3ApplicationConfiguration applicationConfiguration;
   private LibGDXLwjgl3ApplicationAdapter applicationAdapter;
   private Lwjgl3Window window;
   private GenericMouseEventHandler mouseEventHandler;
   private GenericKeyEventHandler keyEventHandler;
   private SpriteBatch spriteBatch;
   private final HashMap<Sprite, Texture> textureMap = new HashMap<>();

   public SpriteWorldViewerUsingLibGDX(String name)
   {
      this.name = name;
   }

   @Override
   public void setPreferredSizeInPixels(int preferredWidth, int preferredHeight)
   {
      this.preferredWidth = preferredWidth;
      this.preferredHeight = preferredHeight;

   }

   @Override
   public void setSpriteWorld(SpriteWorld spriteWorld)
   {
      this.spriteWorld = spriteWorld;

   }

   @Override
   public SpriteWorld getSpriteWorld()
   {
      return spriteWorld;
   }

   @Override
   public void setResizable(boolean resizable)
   {
      this.resizable = resizable;
   }

   @Override
   public void createAndDisplayWindow()
   {
      applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle(name);
      applicationConfiguration.setWindowedMode(preferredWidth, preferredHeight);
      applicationConfiguration.setWindowPosition(startupLocationOnScreenX, startupLocationOnScreenY);
      applicationConfiguration.setBackBufferConfig(8, 8, 8, 8, 16, 0, 1);
      applicationConfiguration.useOpenGL3(true, 3, 3);

      applicationAdapter = new LibGDXLwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            window = ((Lwjgl3Graphics) Gdx.graphics).getWindow();
            mouseEventHandler = new GenericMouseEventHandler(SpriteWorldViewerUsingLibGDX.this, spriteWorld);
            keyEventHandler = new GenericKeyEventHandler(SpriteWorldViewerUsingLibGDX.this, spriteWorld);

            Gdx.input.setInputProcessor(new InputProcessor()
            {
               @Override
               public boolean keyDown(int keycode)
               {
                  keyEventHandler.keyPressed(Input.Keys.toString(keycode));
                  return false;
               }

               @Override
               public boolean keyUp(int keycode)
               {
                  keyEventHandler.keyReleased(Input.Keys.toString(keycode));
                  return false;
               }

               @Override
               public boolean keyTyped(char character)
               {
                  keyEventHandler.keyTyped(String.valueOf(character));
                  return false;
               }

               @Override
               public boolean touchDown(int screenX, int screenY, int pointer, int button)
               {
                  double xWorld = screenX; // FIXME
                  double yWorld = screenY; // FIXME
                  mouseEventHandler.mousePressed(xWorld, yWorld);
                  return false;
               }

               @Override
               public boolean touchUp(int screenX, int screenY, int pointer, int button)
               {
                  double xWorld = screenX; // FIXME
                  double yWorld = screenY; // FIXME
                  mouseEventHandler.mouseReleased(xWorld, yWorld);
                  return false;
               }

               @Override
               public boolean touchDragged(int screenX, int screenY, int pointer)
               {
                  double xWorld = screenX; // FIXME
                  double yWorld = screenY; // FIXME
                  mouseEventHandler.mouseDragged(xWorld, yWorld);
                  return false;
               }

               @Override
               public boolean mouseMoved(int screenX, int screenY)
               {
                  double xWorld = screenX; // FIXME
                  double yWorld = screenY; // FIXME
                  mouseEventHandler.mouseMoved(xWorld, yWorld);
                  return false;
               }

               @Override
               public boolean scrolled(float amountX, float amountY)
               {
                  return false;
               }
            });

            spriteBatch = new SpriteBatch();

            for (Sprite sprite : spriteWorld.getSprites())
            {
               int x = 0;
               int y = 0;
               int width = (int) sprite.getCostume().getImageWidthPixels();
               int height = (int) sprite.getCostume().getImageHeightPixels();;
               WritablePixelFormat<ByteBuffer> pixelFormat = WritablePixelFormat.getByteBgraInstance();
               ByteBuffer byteBuffer = ByteBuffer.allocate(width * height * 4); // TODO: BRGA 8?
               int scanlineStride = (int) sprite.getCostume().getImageWidthPixels(); // TODO: Is this right?
               sprite.getCostume().getImage().getPixelReader().getPixels(x, y, width, height, pixelFormat, byteBuffer, scanlineStride);
               try
               {
                  int requestedFormat = GDX2D_FORMAT_RGBA8888;
                  Pixmap pixmap = new Pixmap(new Gdx2DPixmap(new ByteArrayInputStream(byteBuffer.array()), requestedFormat));
                  textureMap.put(sprite, new Texture(pixmap));
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
         }

         @Override
         public void render()
         {
            Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT); // This cryptic line clears the screen.
            spriteBatch.begin();

            for (Sprite sprite : spriteWorld.getSprites())
            {
               float x = (float) sprite.getX();
               float y = (float) sprite.getY();
               float originX = 0.0f;
               float originY = 0.0f;
               float width = (float) sprite.getWidth();
               float height = (float) sprite.getHeight();
               float scaleX = 1.0f;
               float scaleY = 1.0f;
               float rotation = (float) sprite.getRotationInRadians();
               int srcX = 0;
               int srcY = 0;
               int srcWidth = (int) sprite.getWidth(); // TODO: Check
               int srcHeight = (int) sprite.getHeight(); // TODO: Check
               boolean flipX = false;
               boolean flipY = false;
               spriteBatch.draw(textureMap.get(sprite),
                                x,
                                y,
                                originX,
                                originY,
                                width,
                                height,
                                scaleX,
                                scaleY,
                                rotation,
                                srcX,
                                srcY,
                                srcWidth,
                                srcHeight,
                                flipX,
                                flipY);
            }

            spriteBatch.end();
         }

         @Override
         public void dispose()
         {

         }
      };

      Thread libGDXThread = new Thread(() ->
      {
         new Lwjgl3Application(applicationAdapter, applicationConfiguration);
      }, name);
      libGDXThread.start();
   }

   @Override
   public void addButton(JButton button)
   {

   }

   @Override
   public void addButton(Button button)
   {

   }

   @Override
   public void update()
   {

   }

   @Override
   public void setLocationOnScreen(int x, int y)
   {
      if (window != null)
      {
         window.setPosition(x, y);
      }
      else
      {
         startupLocationOnScreenX = x;
         startupLocationOnScreenY = y;
      }
   }

   @Override
   public void exit()
   {
      Gdx.app.exit();
   }
}
