package frc.robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class MakeButtonList {

	static String controllerNames = "(driverController|operatorController)";
	static HashMap<String, String> buttonNames = new HashMap<String, String>();

	public static void main(String[] args){
		setUpButtonNames();
		try {
			String fileContents = Files.readString(Paths.get("src/main/java/frc/robot/RobotContainer.java"));
			Pattern classPattern = Pattern.compile("\\{(.*)\\}$", Pattern.DOTALL);
			Matcher classMatcher = classPattern.matcher(fileContents);
			ArrayList<String> controlList = new ArrayList<String>();

			if(classMatcher.find()){
				String classBody = classMatcher.group(0);
				classBody = classBody.replaceAll("(?s)\\/\\*(.*?)\\*\\/", "");
				classBody = classBody.replaceAll("(?m)//.*$", "");
				classBody = classBody.replaceAll("(\\s\\s)+", "");
				classBody = classBody.replaceAll("\\n", "");
				classBody = classBody.replaceAll("\\r", "");
				classBody = classBody.replaceAll("\\t", "");
				Pattern functionPattern = Pattern.compile("\\{([^\\{]*[^\\}])\\}");
				Matcher functionMatcher = functionPattern.matcher(classBody);
				while(functionMatcher.find()){
					String functionBody = functionMatcher.group(1);
					String[] lines = functionBody.split(";");
					for(String s : lines){
						// System.out.println(s);
						Matcher controllerMatcher = Pattern.compile(controllerNames).matcher(s);
						if(controllerMatcher.find()){
							String controller = controllerMatcher.group(1);
							Matcher buttonMatcher = Pattern.compile("get(.*?)\\((.*?)\\)\\.").matcher(s);
							if(buttonMatcher.find()){
								String button = buttonMatcher.group(1);
								String subbutton = buttonMatcher.group(2);
								Matcher triggerMatcher = Pattern.compile(button + "\\(" + subbutton.replace(".", "\\.") + "\\)\\.(.*?)\\(").matcher(s);
								// System.out.println(button + "\\(" + subbutton.replace(".", "\\.") + "\\)\\.(.*?)\\(");
								if(triggerMatcher.find()){
									String trigger = triggerMatcher.group(1);

									Matcher commandMatcher = Pattern.compile("\\(\\s+new ([a-zA-Z]+)\\((.*)\\)\\s+\\)").matcher(s);
									if(commandMatcher.find()){
										String action = commandMatcher.group(1);
										// String command = commandMatcher.group(2);
										String commandArgsS = commandMatcher.group(2);
										// System.out.printf("%s %s %s %s %s %s\n", getFormattedButtonString(controller), getFormattedButtonString(trigger), getFormattedButtonString(button), getFormattedButtonString(action), "command", commandArgsS);

										String[] commandArgs = commandArgsS.replace(" ", "").split(",");
										ArrayList<String> commandsss = new ArrayList<String>();
										for(String arg : commandArgs){
											if((!arg.startsWith("m_") && !arg.endsWith("Subsystem")) || arg.contains("::") ){
												commandsss.add(arg);
											}
										}
										if(commandsss.size() > 0){
											String finalArgs = String.join(", ", commandsss);
											String finalFunction = String.format("%s, %s %s, %s (%s)", getFormattedButtonString(controller), getFormattedButtonString(button).replace("***", getFormattedButtonString(subbutton)), getFormattedButtonString(trigger).replace("getButton", ""), getFormattedButtonString(action), finalArgs);
											controlList.add(finalFunction);
										} else {
											String finalFunction = String.format("%s, %s %s, %s", getFormattedButtonString(controller), getFormattedButtonString(button).replace("***", getFormattedButtonString(subbutton)), getFormattedButtonString(trigger).replace("getButton", ""), getFormattedButtonString(action));
											controlList.add(finalFunction);
										}
										
									}
								}
							}
						}
					}
				}

				// Print file
				BufferedWriter output = new BufferedWriter(new FileWriter("buttons.txt"));
				// output.write("|Controller|Button|Action|\r\n");
				// output.write("|-|-|-|\r\n");
				// Collections.sort(controlList);	
				Iterator<String> i = controlList.iterator();
				while(i.hasNext()){
					output.write(i.next());
					output.write("\r\n");
				}
				output.close();
				System.out.println("Button mapping file generated! Saved to ${rootDir}/buttons.txt");
			}
		} catch(Exception e){
			e.printStackTrace();
		}
	}

	static String getFormattedButtonString(String name){
		name = name.replace(" ", "");
		if(buttonNames.containsKey(name)){
			return buttonNames.get(name);
		} else return name;
	}

	public static void setUpButtonNames(){
		// Controllers
		buttonNames.put("driverController", "Driver Controller");
		buttonNames.put("operatorController", "Operator Controller");

		// Buttons/triggers
		buttonNames.put("JoystickButton", "*** Button");
		buttonNames.put("POVButton", "D-Pad ***");
		buttonNames.put("StickLeft", "Left Stick");
		buttonNames.put("StickRight", "Right Stick");
		buttonNames.put("BumperLeft", "Left Bumper");
		buttonNames.put("BumperRight", "Right Bumper");
		buttonNames.put("AButton", "A Button");
		buttonNames.put("BButton", "B Button");
		buttonNames.put("XButton", "X Button");
		buttonNames.put("YButton", "Y Button");
		buttonNames.put("LeftBumperButton", "Left Bumper");
		buttonNames.put("RightBumperButton", "Right Bumper");
		buttonNames.put("LeftTriggerAxis", "Left Trigger");
		buttonNames.put("RightTriggerAxis", "Right Trigger");
		buttonNames.put("DPadButton", "D-Pad ***");

		// Trigger actions
		buttonNames.put("whileHeld", "(Hold)");
		buttonNames.put("whenHeld", "(Hold, interruptable)");
		buttonNames.put("toggleWhenPressed", "(Toggle on/off)");
		buttonNames.put("whileActiveOnce", "(Hold, interruptable)");
		buttonNames.put("whileActiveContinuous", "(Hold)");
		buttonNames.put("whenReleased", "(When button is released)");
		buttonNames.put("whenPressed", "(Press)");
		buttonNames.put("whenInactive", "(Runs when not held)");
		buttonNames.put("whenActive", "(When activated)");
		buttonNames.put("toggleWhenActive", "(Toggles on/off)");
		buttonNames.put("cancelWhenPressed", "(Cancels command when pressed)");
		buttonNames.put("cancelWhenActive", "(Cancels command when activated)");

		// D-Pad POV angles
		buttonNames.put("0", "Up");
		buttonNames.put("90", "Left");
		buttonNames.put("180", "Down");
		buttonNames.put("270", "Right");
		buttonNames.put("Direction.UP", "Up");
		buttonNames.put("Direction.UPRIGHT", "Up-Right");
		buttonNames.put("Direction.RIGHT", "Right");
		buttonNames.put("Direction.DOWNRIGHT", "Down-Right");
		buttonNames.put("Direction.DOWN", "Down");
		buttonNames.put("Direction.DOWNLEFT", "Down-Left");
		buttonNames.put("Direction.LEFT", "Left");
		buttonNames.put("Direction.UPLEFT", "Up-Left");
	}
}