/*
    MPU Teapot example
    
    This code will display airplane graphs that will follow the MPU6050 movements.

    Use #define OUTPUT_TEAPOT output definition to make this code work.

    Define the serial port in the code below (line 45). Note: Close any other serial instance using the port.

    NOTE:  ToxicLibs library is required.
    1. Download from https://github.com/postspectacular/toxiclibs/releases
    2. Extract into [userdir]/Documents/Processing/libraries (location may be different on Mac/Linux)
    3. Restart Processing if needed 
*/

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

void setup() {
    size(300, 300, OPENGL); // 300px square viewport using OpenGL rendering
    gfx = new ToxiclibsSupport(this);

    /* Setup lights and antialiasing */
    lights();
    smooth();
  
    println(Serial.list()); //Display serial port list for debugging/clarity
    String portName = "COM8"; //Define the port, port format may be different on Linux/Mac
    port = new Serial(this, portName, 115200); // Open the serial port
    port.write('r'); // Send a single character to trigger DMP init/start
}

void draw() {
    if (millis() - interval > 1000) {
        /*Resend a single character to trigger DMP init/start
        in case the MPU is halted/reset while the applet is running*/
        port.write('r');
        interval = millis();
    }
    
    background(0);  // Black background
    pushMatrix();
    translate(width / 2, height / 2);   //Translate everything to the middle of the viewport

    /* Toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    different coordinate system orientation assumptions between Processing
    and InvenSense DMP)*/
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);

    /*Drawing the airplane*/
    fill(255, 0, 0, 200);   //Draw main body in red
    box(10, 10, 200);
    fill(0, 0, 255, 200);   //Draw front-facing tip in blue
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();
    fill(0, 255, 0, 200);   //Draw wings and tail fin in green
    beginShape(TRIANGLES);
    vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);  //Wing top layer
    vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);  //Wing bottom layer
    vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);  //Tail left layer
    vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);  //Tail right layer
    endShape();
    beginShape(QUADS);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    endShape();
    
    popMatrix();
}

void serialEvent(Serial port) {
    interval = millis();
    
    // Process while there is enough raw data to form at least one full teapot packet (14 bytes)
    while (port.available() >= 14) {
        int ch = port.read();

        // 1. Hunt for the start packet sequence '$' followed by 0x02
        if (ch == '$') {
            int checkType = port.read();
            if (checkType == 2) {
                
                // 2. We have a confirmed header match. Read the remaining 12 data bytes immediately.
                byte[] dataBuffer = new byte[12];
                port.readBytes(dataBuffer);
                
                // 3. Perform a footer verification to confirm structural packet alignment
                if (dataBuffer[10] == '\r' && dataBuffer[11] == '\n') {
                    
                    // 4. Reconstruct signed 16-bit short integers from individual byte indices safely
                    short q0_raw = (short)((dataBuffer[0] << 8) | (dataBuffer[1] & 0xFF));
                    short q1_raw = (short)((dataBuffer[2] << 8) | (dataBuffer[3] & 0xFF));
                    short q2_raw = (short)((dataBuffer[4] << 8) | (dataBuffer[5] & 0xFF));
                    short q3_raw = (short)((dataBuffer[6] << 8) | (dataBuffer[7] & 0xFF));
                    
                    // 5. Convert raw bits into floating-point fractional Quaternions
                    q[0] = q0_raw / 16384.0f;
                    q[1] = q1_raw / 16384.0f;
                    q[2] = q2_raw / 16384.0f;
                    q[3] = q3_raw / 16384.0f;
                    
                    // 6. Normalize vector space to keep rotations rock-solid and freeze-proof
                    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
                    if (norm > 0) {
                        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
                    }

                    // Push mathematically sound values straight to the visualizer matrix
                    quat.set(q[0], q[1], q[2], q[3]);   
                }
            }
        }
    }
}



void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
    
        // Center point
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
}
