#include <Wire.h>

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(57600);
  set_registers();
  Serial.println("MPU-6050");
  Serial.println("Caliberation Sequence Initiated");

  for(int cal = 0; cal < 2000; cal++)
  {
    mpu_read();
    gyro_x_cal += gyro_x;                                              
    gyro_y_cal += gyro_y;                                              
    gyro_z_cal += gyro_z;                                              
    delay(3); // To stimulate  250Hz program
  }
  Serial.println("Caliberation Completed !");
  
  gyro_x_cal /= 2000;                                                  
  gyro_y_cal /= 2000;                                                  
  gyro_z_cal /= 2000;     

  loop_timer = micros();  
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu_read();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  //0.0000611 = 1 / (250Hz / 65.5) == Gyro Angle Calculations
  angle_pitch += gyro_x * 0.0000611;
  angle_roll += gyro_y * 0.0000611;
  // 0.0000611 * (3.142(PI) / 180(degereeesss) = 0.000001066
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer Calculationsssss
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  //57.296 = 1 / (3.142 / 180)
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
  
  angle_pitch_acc -= 0.0;                                              
  angle_roll_acc -= 0.0;

  if(set_gyro_angles){
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  } else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }

  // Complementary Filter
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      // Vice Versha

  Serial.print("Pitch : ");
  Serial.print(angle_pitch_output);
  Serial.print(" Roll : ");
  Serial.println(angle_roll_output);

  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();   
}

void set_registers()
{
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                    
  Wire.endTransmission();                                              
  //Configure the accelerometer
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                    
  Wire.write(0x10);                                                    
  Wire.endTransmission();                                              
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                    
  Wire.endTransmission();     
}

void mpu_read()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x6B, 14); //14 Bytes
  while( Wire.available() < 14 );
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temperature = Wire.read()<<8|Wire.read();                            
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}
