/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TalBotAutoOpMode extends TalBotOpMode {
	String colorString;

	/**
	 * Constructor
	 */
	public TalBotAutoOpMode() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		super.init();
	}

	/*
	 * This method will be called repeatedly in a loop
	 */
	@Override
	public void loop() {

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 */
	@Override
	public void stop() {

	}



	//Methods for detecting light color
	public String getColor(){
		int red = colorSensor.red();
		int green = colorSensor.green();
		int blue = colorSensor.blue();


		if (isWhite(red, green, blue)){
			return "white";
		}
		else if (isBlue(red, green, blue)){
			return "blue";
		}
		return "gray";

	}

	private boolean isWhite(int red, int green, int blue){
		//white - r,g,b must all be > 90 and close to another
		if (red > 90 && green > 90 && blue > 90){
			double one = (red - green)/((red + green)/2.0);
			double two = (red - blue)/((red + blue)/2.0);
			double three = (blue - green)/((blue + green)/2.0);
			if (Math.abs(one) < .2 && Math.abs(two) < .2 && Math.abs(three) < .2){
				return true;
			}
		}

		return false;
	}

	private boolean isBlue(int red, int green, int blue){
		//blue must be > 40% other 2 colors
		double one = (blue - red)/((double)red);
		double two = (blue - green)/((double)green);
		return (one > .4 && two > .4);
	}


}
