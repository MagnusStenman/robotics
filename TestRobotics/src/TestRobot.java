import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.util.Map;

// Jar file for JSON support
import com.fasterxml.jackson.core.*;
import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.annotation.*;

/**
 * TestRobot interfaces to the (real or virtual) robot over a network connection.
 * It uses Java -> JSON -> HttpRequest -> Network -> DssHost32 -> Lokarria(Robulab) -> Core -> MRDS4
 * 
 * @author thomasj
 */
public class TestRobot
{
   private String host;                // host and port numbers
   private int port;
   private ObjectMapper mapper;        // maps between JSON and Java structures
   private TestRobot robot;

   /**
    * Create a robot connected to host "host" at port "port"
    * @param host normally http://127.0.0.1
    * @param port normally 50000
    */
   public TestRobot(String host, int port)
   {
      this.host = host;
      this.port = port;
      this.robot = this;

      mapper = new ObjectMapper();
   }

   /**
    * This simple main program creates a robot, sets up some speed and turning rate and
    * then displays angle and position for 16 seconds.
    * @param args         not used
    * @throws Exception   not caught
    */
   public static void main(String[] args) throws Exception
   {     
      System.out.println("Creating Robot");
      TestRobot robot = new TestRobot("http://127.0.0.1", 50000);

      System.out.println("Creating response");
      LocalizationResponse lr = new LocalizationResponse();

      System.out.println("Creating request");
      DifferentialDriveRequest dr = new DifferentialDriveRequest();

      // set up the request to move in a circle
      dr.setAngularSpeed(Math.PI * 0.25);
      dr.setLinearSpeed(1.0);

      System.out.println("Start to move robot");
      int rc = robot.putRequest(dr);
      System.out.println("Response code " + rc);

      for (int i = 0; i < 16; i++)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex) {}

         // ask the robot about its position and angle
         robot.getResponse(lr);

         double angle = robot.getBearingAngle(lr);
         System.out.println("bearing = " + angle);

         double [] position = robot.getPosition(lr);
         System.out.println("position = " + position[0] + ", " + position[1]); 
      }

      // set up request to stop the robot
      dr.setLinearSpeed(0);
      dr.setAngularSpeed(0);

      System.out.println("Stop robot");
      rc = robot.putRequest(dr);
      System.out.println("Response code " + rc);

   }

   /**
    * Extract the robot bearing from the response
    * @param lr
    * @return angle in degrees
    */
   double getBearingAngle(LocalizationResponse lr)
   {
      double e[] = lr.getOrientation();

      double angle = 2 * Math.atan2(e[3], e[0]);
      return angle * 180 / Math.PI;
   }

   /**
    * Extract the position
    * @param lr
    * @return coordinates
    */
   double[] getPosition(LocalizationResponse lr)
   {
      return lr.getPosition();
   }
   

   /**
    * Send a request to the robot.
    * @param r request to send
    * @return response code from the connection (the web server)
    * @throws Exception
    */
   public int putRequest(Request r) throws Exception
   {
      URL url = new URL(host + ":" + port + r.getPath());

      HttpURLConnection connection = (HttpURLConnection)url.openConnection();

      connection.setDoOutput(true);

      connection.setRequestMethod("POST");
      connection.setRequestProperty("Content-Type", "application/json");
      connection.setUseCaches (false);

      OutputStreamWriter out = new OutputStreamWriter(
            connection.getOutputStream());

      // construct a JSON string
      String json = mapper.writeValueAsString(r.getData());

      // write it to the web server
      out.write(json);
      out.close();

      // wait for response code
      int rc = connection.getResponseCode();

      return rc;
   }

   /**
    * Get a response from the robot
    * @param r response to fill in
    * @return response same as parameter
    * @throws Exception
    */
   public Response getResponse(Response r) throws Exception
   {
      URL url = new URL(host + ":" + port + r.getPath());
      System.out.println(url);

      // open a connection to the web server and then get the resulting data
      URLConnection connection = url.openConnection();
      BufferedReader in = new BufferedReader(new InputStreamReader(
            connection.getInputStream()));

      // map it to a Java Map
      Map<String, Object> data = mapper.readValue(in, Map.class);
      r.setData(data);

      in.close();

      return r;
   }

}

