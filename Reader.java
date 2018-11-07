package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.app.Application;
import android.content.Context;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.Charset;
import java.util.Arrays;
import java.util.Scanner;

public class Reader {
	
	public Reader() {
		
	}
	
	public Segment[] getSegments(int file, Telemetry telemetry, Context context) {

		int rows = 0;
		InputStream inputStream = context.getResources().openRawResource(file);
		BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream, Charset.forName("UTF-8")));
		Segment[] segments = new Segment[1024];

		String input = "";
		try {

			reader.readLine();

			while((input = reader.readLine()) != null) {

				String[] inputArray = input.split(",");
				segments[rows] = new Segment(inputArray);
				rows++;
				
			}
			
		}catch(Exception ex) {
			
			telemetry.addData("Exception Reading", ex.getMessage());
			
		}
		
		segments = Arrays.copyOf(segments, rows);
		
		return segments;
		
	}

}
