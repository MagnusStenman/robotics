package Robot;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;

import SuppliedFiles.LocalizationResponse;

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
		
		LekRobot robot = new LekRobot("http://127.0.0.1", 50000, "testPath.json");
		robot.start();
		
	}

	public List<Map<String, Object>> readFile(String filePath) throws JsonParseException, JsonMappingException, IOException {
	   File file = new File(filePath);
	   return mapper.readValue(file, TypeFactory.defaultInstance().constructCollectionType(List.class, Map.class));
	}
	
	public void start() {
		for (int i=0;i<mapList.size();i++) {
			System.out.println(mapList.get(i));
		}
	}
}
