package Robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.util.List;
import java.util.Map;

import SuppliedFiles.DifferentialDriveRequest;
import SuppliedFiles.LocalizationResponse;
import SuppliedFiles.Position;
import SuppliedFiles.Request;
import SuppliedFiles.Response;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.TypeFactory;

public class LekRobot {

	private String host;
	private int port;
	private LekRobot robot;
	private ObjectMapper mapper;
	private List<Map<String, Object>> mapList;

	public LekRobot(String host, int port) {
		this.host = host;
		this.port = port;
		this.robot = this;

		mapper = new ObjectMapper();
	}

	public LekRobot(String host, int port, String filePath) {
		this.host = host;
		this.port = port;
		this.robot = this;

		mapper = new ObjectMapper();
		try {
			mapList = readFile(filePath);
		} catch (JsonParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (JsonMappingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void main(String[] args) {

		LekRobot robot = new LekRobot("http://127.0.0.1", 50000,
				"testPath.json");
		try {
			robot.start();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public List<Map<String, Object>> readFile(String filePath)
			throws JsonParseException, JsonMappingException, IOException {
		File file = new File(filePath);
		return mapper.readValue(file, TypeFactory.defaultInstance()
				.constructCollectionType(List.class, Map.class));
	}

	public void start() throws Exception {

		/* 
		 * Is path defined? if yes continue with algorithm
		 * 
		 * 1. check robot position
		 * 2. check length and angle towards next position
		 * 3. calculate angular and linear speed
		 * 4. run for calculated seconds
		 * 
		 * (loop) check #2 again to see if we are close to target
		 * 		- if close track next target position and run loop
		 * 		- if not -> continue loop (1 -> 2 -> 3 -> 4)
		 * 
		 */
		
		int i=0;
		do {
			
			LocalizationResponse robotLR = new LocalizationResponse();
			LocalizationResponse nextLR = new LocalizationResponse();
			nextLR.setData(mapList.get(i));
			robot.getResponse(robotLR);
			Position robotPos = new Position(robotLR.getPosition());
			Position nextPos = new Position(nextLR.getPosition());
			
			double targetDistance = robotPos.getDistanceTo(nextPos);
			double targetAngle = robotPos.getBearingTo(nextPos);
			
			System.out.println("robotPOS: " + robotPos.getX() + ", " + robotPos.getY());
			System.out.println("nextPOS: " + nextPos.getX() + ", " + nextPos.getY());
			System.out.println("target distance " + targetDistance);
			System.out.println("target angle " + targetAngle);
			
			i++;
			
		} while (mapList.size() > i);
		
		//TESTCODE//
		/*
		DifferentialDriveRequest ddr = new DifferentialDriveRequest();
		
		LocalizationResponse lr = new LocalizationResponse();
		robot.getResponse(lr);
		
		if (robot.getBearingAngle(lr) < 0) {
			
		}
		
		System.out.println(robot.getBearingAngle(lr));
		
		ddr.setAngularSpeed(0.2);
		putRequest(ddr);
		try {
			Thread.sleep(10000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		ddr.setAngularSpeed(0);
		putRequest(ddr);
		*/
		//TESTCODE//
		
	}

	/**
	 * Send a request to the robot.
	 * 
	 * @param r
	 *            request to send
	 * @return response code from the connection (the web server)
	 * @throws Exception
	 */
	public int putRequest(Request r) throws Exception {
		URL url = new URL(host + ":" + port + r.getPath());

		HttpURLConnection connection = (HttpURLConnection) url.openConnection();

		connection.setDoOutput(true);

		connection.setRequestMethod("POST");
		connection.setRequestProperty("Content-Type", "application/json");
		connection.setUseCaches(false);

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
	 * 
	 * @param r
	 *            response to fill in
	 * @return response same as parameter
	 * @throws Exception
	 */
	public Response getResponse(Response r) throws Exception {
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
	
	double getBearingAngle(LocalizationResponse lr) {
		double e[] = lr.getOrientation();

		double angle = 2 * Math.atan2(e[3], e[0]);
		double positiveAngle = angle * 180 / Math.PI;
		if (positiveAngle < 0) {
			positiveAngle += 360;
		}
		return positiveAngle;
	}
}
