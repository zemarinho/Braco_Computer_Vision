#include <Servo.h>
#include <SoftwareSerial.h>


#define RxMotorsPin 2
#define TxMotorsPin 3
#define clawServoPin 9

#define default_pos_x 0.10
#define default_pos_y 0.00
#define default_pos_z 0.15  
#define default_pos_tool_pos 0
#define default_pos_tool_angle -90

#define NEW_POS_LIMIT 0.02
#define ROTATION_TIME 2

#define update_pos_rate 1

/*IDENTIFICAÇÃO DOS SERVOS DE BAIXO PARA CIMA:
  - SERVO 0: base, responsável pela rotação no plano XY;
  - SERVOS 1 e 2: meio, responsáveis pela articulação do braço e colocação do pulso na posição desejada controlando a altura e alongamento do braço
  - SERVO 3: pulso, responsável pelo angulo de inclinação da garra
  - SERVO 4: tool, controla fecho e abertura da garra*/

float b0 = 0.105; //comprimento de cada elo do braço
float b1 = 0.13;
float b2 = 0.13;
float b3 = 0.001;



//Posição central de cada servo
int theta0_center = 500;
int theta1_center = 550;
int theta2_center = 470;
int theta3_center = 480;
int theta4_center = 500;

//range de 240 (+-120 para cada lado); o centro é aproxcimadamente 0 e écoincidente com a posição central de cada servo
float theta0_min = -120 + (theta0_center-500)*240/1000;
float theta0_max = 120 - (500-theta0_center)*240/1000;
float theta1_min = -120 + (theta1_center-500)*240/1000;
float theta1_max = 120 - (500-theta1_center)*240/1000;
float theta2_min = -120 + (theta2_center-500)*240/1000;
float theta2_max = 120 - (500-theta2_center)*240/1000;
float theta3_min = -120 + (theta3_center-500)*240/1000;
float theta3_max = 120 - (500-theta3_center)*240/1000;

SoftwareSerial motorsSerial (RxMotorsPin, TxMotorsPin);
Servo clawServo;

float theta0, theta1, theta2, theta3, theta4;

float goal_x = default_pos_x, goal_y = default_pos_y, goal_z = default_pos_z; //posição objetivo em x, y e z
float prev_goal_x = 0, prev_goal_y = 0, prev_goal_z = 0; //variável com a posição anterior em x, y e z
float prev_theta0 = 0, prev_theta1 = 0, prev_theta2 = 0, prev_theta3 = 0; //variável com a posição anterior em angulo de cada servo
float goal_tool_pos = default_pos_tool_pos,goal_tool_angle = default_pos_tool_angle; //posição objetivo e ângulo objetivo da garra

float recv_x, recv_y, recv_z;


int inverse_kinematics_pulse(float x, float y, float z, float angle, float *pulse_x, float *pulse_y, float *pulse_z){
  /*
    recebe as coordenadas da posição objetivo para a garra
    e determina as coordenadas da posição objetivo do pulso
    PULSE == WRIST, servo que controla o angulo da garra
  */
  float goal_tool_r = sqrt(x*x + y*y); //distância-objetivo ao centro (coordenadas cilindricas) da garra
  
  float goal_pulse_r = goal_tool_r - b3*cos(angle*3.1416/180);/*determinar a distância ao centro do pulso tendo em conta a posição desejada para a garra
                                                                (goal_tool_r), do comprimento da ligação do pulso (b3), e o angulo em que a garra faz 
                                                                com a vertical (ou horizontal, idk) (angle, dado em graus)*/

  float goal_pulse_t0 = atan2(y,x); //ângulo em radianos em em que o braço está a apontar segundo o plano XY mediante uma referência

  if (goal_pulse_t0<-1 || goal_pulse_t0 > 1) return 1; //limitador de posições impossíveis
  
  //fazer o retorno da função nas variáveis passadas como parâmetros da função
  *pulse_x = goal_pulse_r * cos(goal_pulse_t0); //posição em X do pulso
  *pulse_y = goal_pulse_r * sin(goal_pulse_t0); //posição em Y do pulso
  *pulse_z =  z - b3*sin(angle*3.1416/180); //posição em Z do pulso

  return 0;
}
int direct_kinematics(float theta0, float theta1, float theta2, float theta3,float *x, float *y, float *z, float angle){
  /*
    recebe o angulo dos 4 primeiros servos
    e determina a posição em que está o pulso
    com coordenadas X, Y e Z e retorna os valores
    nas variáveis passadas como parâmetro
  */

  float rb,zb;

  zb= b1*sin(theta1) + b2*sin(theta1 + theta2); //altura a que e está o pulso
  rb= b1*cos(theta1) + b2*cos(theta1 + theta2); //distância a que o pulso está dpo centro

  *z = zb; //altura do pulso
  *y = rb * sin(theta0); //posição Y do pulso
  *x = rb * cos(theta0); //posição X do pulso
  return 0;
}

int inverse_kinematics(float x, float y, float z, float angle, float *theta0, float *theta1, float *theta2, float *theta3){
  /*
    recebe as coordenadas da posição
    desejada do PULSO e calcula o ângulo
    em que deve ser colocado cada um dos 3 primeiros servos

    recebe o ângulo em que deve estar a garra (angle)
    e calcula o ângulo em que deve ser colocado o SERVO3
  */
  float angle_aux;
  float r = sqrt(x*x+y*y); //distância ao centro (coordenadas cilíndricas) do pulso
  float d = sqrt(z*z+r*r); //distância ao centro (coordenadas esféricas) do pulso
  float pre_t0, pre_t1,pre_t2;
  
  pre_t0 = atan2(y,x); //ângulo em radianos em em que o braço está a apontar segundo o plano XY mediante uma referência

  angle_aux = (b2*b2-b1*b1-d*d)/(-2*b1*d); //cosseno do angulo vertical (coordenadas cilíndricas) oposto ao lado b2 no triângulo feito por b1, b2 e d
  if (angle_aux<-1 || angle_aux >1){ //deteta posilções impossíveis
    return 1;
  }

  float gamma1 = acos(angle_aux); //angulo vertical (coordenadas cilíndricas) do angulo oposto ao lado b2 no triângulo [b1,b2,d]

  pre_t1 = atan2(z,r) + gamma1; /*angulo em que deve ficar o SERVO1 (angulo vertical coordenadas cilindricas
                                  daposição do pulso + angulo oposto ao lado b2 no triângulo [b1,b2,d])*/
                                //varia de 0 a pi/2

  angle_aux = (d*d-b1*b1-b2*b2)/(-2*b1*b2); //cosseno do angulo realizado pelo SERVO2
  if (angle_aux<-1 || angle_aux >1){ //deteta posições impossíveis
    return 2;
  }

  pre_t2 = acos(angle_aux)-3.1416; //angulo do SERVO2 (varia de -pi/2 a pi/2)


  *theta0 = pre_t0; //atribuição os valores de cada ângulo
  *theta1 = pre_t1; //às respetivas variáveis de retorno
  *theta2 = pre_t2; //para cada servo
  *theta3 = angle*3.1416/180 - pre_t1 - pre_t2; //angulo do SERVO3
  return 0;
}

void init_motors(){ //inicializa os servos
  pinMode(RxMotorsPin, INPUT);
  pinMode(TxMotorsPin, OUTPUT);
  motorsSerial.begin(9600);

  clawServo.attach(9);
}

// Function to send data in the specified format
void sendData(byte data[], int length) {
  for (int i = 0; i < length; i++) {
    motorsSerial.write(data[i]);
  }
}

// Function to create and send a command for servo movement
void sendServoMoveCommand(byte servoID, int angle, int time) {
  byte data[10];

  // Frame header
  data[0] = 0x55;
  data[1] = 0x55;

  // Calculate data length
  data[2] = 0x08;

  // Command byte
  data[3] = 0x03; // CMD_SERVO_MOVE

  // Angle (high and low byte)
  data[4] = 0x01;         // Low byte
  data[5] = time & 0xFF;  // High byte

  // Set servo ID
  data[6] = (time >> 8) & 0xFF;
  data[7] = servoID;  //ID
  data[8] = angle & 0xFF;  // High byte of time
  
  data[9] = (angle >> 8) & 0xFF;

  // Send the data
  sendData(data, 10);
}

//limtador de velocidade para o movimento de cada motor
void vLimiter(float old_t_0, float old_t_1, float old_t_2, float old_t_3,
              float new_t_0, float new_t_1, float new_t_2, float new_t_3,
              int *time0, int *time1, int *time2, int *time3){
  /*
  theta é dado entre 0 e 1000, correspondente de 0º a 240º
  Vmax do servo = 300º/s
  Vsegura (até ver):
  * 33%-50% SERVO0 -->>  100-150
  * 27%-33% SERVO1 SERVO2 -->> 80-100
  * 17%-33% SERVO3 -->> 50-100
  V = alfa/t => t = alfa/V
  retorno do tempo em milissegundos
  */
  *time0 =(int) (((abs(old_t_0-new_t_0)*240/1000 /*angulo de deslocamento em graus*/)/150)*1000);
  *time1 =(int) (((abs(old_t_1-new_t_1)*240/1000)/100)*1000);
  *time2 =(int) (((abs(old_t_2-new_t_2)*240/1000)/100)*1000);
  *time3 =(int) (((abs(old_t_3-new_t_3)*240/1000)/100)*1000);
}



void setup(){
  Serial.begin(115200);
  init_motors();
  Serial.setTimeout(100);
}


unsigned long last_update=0;
void loop() {
  delay(1000/update_pos_rate);
  /*
  delay(1000);
  inverse_kinematics(goal_x, goal_y, goal_z, default_pos_tool_angle, &theta0, &theta1, &theta2, &theta3);

  float x_test,y_test,z_test;
  direct_kinematics(theta0, theta1, theta2, theta3,&x_test,&y_test,&z_test, 0);

  if (sqrt((goal_x-prev_goal_x)*(goal_x-prev_goal_x) + (goal_y-prev_goal_y)*(goal_y-prev_goal_y) + (goal_z-prev_goal_z)*(goal_z-prev_goal_z)) > NEW_POS_LIMIT){
  int theta0_servo = (int)constrain(theta0_center + theta0*1000/(240*3.1416/180),0, 1000);
  int theta1_servo = (int)constrain(theta1_center - (theta1-3.1416/2)*1000/(240*3.1416/180),0, 1000);
  int theta2_servo = (int)constrain(theta2_center + theta2*1000/(240*3.1416/180),0, 1000);
  int theta3_servo = (int)constrain(theta3_center + theta3*1000/(240*3.1416/180),0, 1000);

  sendServoMoveCommand(0x01, theta0_servo, 0.5/update_pos_rate);
  sendServoMoveCommand(0x02, theta1_servo, 0.5/update_pos_rate);
  sendServoMoveCommand(0x03, theta2_servo, 0.5/update_pos_rate);
  sendServoMoveCommand(0x04, theta3_servo, 0.5/update_pos_rate);}
  Serial.print("Angles:");
  Serial.print(theta0*180/3.1416);
  Serial.print("\t ");
  Serial.print(theta1*180/3.1416);
  Serial.print("\t ");
  Serial.print(theta2*180/3.1416);
  Serial.print("\t ");
  Serial.println(theta3*180/3.1416);
  Serial.print("Pos:");
  Serial.print(x_test);
  Serial.print("\t ");
  Serial.print(y_test);
  Serial.print("\t ");
  Serial.println(z_test);*/



  //recvWithEndMarker();
  //msg[0]='\0';
  int i=0;
  /*while (Serial.available() > 0) {
    delay(3);
    msg[i]=Serial.read();
    i++;
  }*/
  //if (i>3)
    //Serial.println(msg);
  //if (msg[0] == '\0' && msg[i-1] == '\n')
  Serial.println(Serial.readStringUntil('\n'));
  while (Serial.available()<10){}


  

    //delay(100);
    //token =strtok(msg,s);
    //Serial.println(token);
    // look for the next valid position in the incoming serial stream:
    float recv_x = atof(Serial.readStringUntil(' ').c_str());
    //token = strtok(NULL, s);
    //Serial.println(token);
    float recv_y =  atof(Serial.readStringUntil(' ').c_str());
    
    //token = strtok(NULL, s);
    //Serial.println(token);
    float recv_z =  atof(Serial.readStringUntil(' ').c_str());
    //token = strtok(NULL, s);
    //Serial.println(token);
    float recv_tool_pos =  atof(Serial.readStringUntil('\n').c_str());

      goal_x = recv_x;
      goal_y = recv_y;
      goal_z = recv_z;


    Serial.print("Pos:");
    Serial.print(recv_x);
    Serial.print("\t ");
    Serial.print(recv_y);
    Serial.print("\t ");
    Serial.println(recv_z);

    /*
    inverse_kinematics_pulse(recv_x, recv_y, recv_z, default_pos_tool_angle, &goal_x, &goal_y, &goal_z);
    Serial.println("Pos on pulse IK");
    Serial.print("Pos:");
    Serial.print(goal_x);
    Serial.print("\t ");
    Serial.print(goal_y);
    Serial.print("\t ");
    Serial.println(goal_z);
    */
    
    /*if (inverse_kinematics_pulse(recv_x, recv_y, recv_z, default_pos_tool_angle, &goal_x, &goal_y, &goal_z)){
        Serial.println("Error on pulse IK");
          Serial.print("Pos:");
          Serial.print(goal_x);
          Serial.print("\t ");
          Serial.print(goal_y);
          Serial.print("\t ");
          Serial.println(goal_z);
        
    }
    else*/ 
    if (inverse_kinematics(goal_x, goal_y, goal_z, default_pos_tool_angle, &theta0, &theta1, &theta2, &theta3)){
          Serial.println("Error on arm IK");
          Serial.print("Pos:");
          Serial.print(goal_x);
          Serial.print("\t ");
          Serial.print(goal_y);
          Serial.print("\t ");
          Serial.println(goal_z);

          
    }
    else{
          /*float x_test,y_test,z_test;
          direct_kinematics(theta0, theta1, theta2, theta3,&x_test,&y_test,&z_test, 0);

          Serial.print("Pos:");
          Serial.print(x_test);
          Serial.print("\t ");
          Serial.print(y_test);
          Serial.print("\t ");
          Serial.println(z_test);*/

          Serial.print("Pos OK:");
          Serial.print(goal_x);
          Serial.print("\t ");
          Serial.print(goal_y);
          Serial.print("\t ");
          Serial.println(goal_z);

          //if (sqrt((goal_x-prev_goal_x)*(goal_x-prev_goal_x) + (goal_y-prev_goal_y)*(goal_y-prev_goal_y) + (goal_z-prev_goal_z)*(goal_z-prev_goal_z)) > NEW_POS_LIMIT)
          {
              int theta0_servo = (int)constrain(theta0_center + theta0*1000/(240*3.1416/180),0, 1000);
              int theta1_servo = (int)constrain(theta1_center - (theta1-3.1416/2)*1000/(240*3.1416/180),0, 1000);
              int theta2_servo = (int)constrain(theta2_center + theta2*1000/(240*3.1416/180),0, 1000);
              int theta3_servo = (int)constrain(theta3_center + theta3*1000/(240*3.1416/180),0, 1000);
              int theta4_servo = map((int)(recv_tool_pos*1000), 0, 250, 0, 90);
              if (theta4_servo > 90)
                theta4_servo = 90;
              if (theta4_servo < 0)
                theta4_servo = 0;
              Serial.println(theta4_servo);
            
            //CALCULADOR DE TEMPO PARA LIMITAR VELOCIDADE DE MOVIMENTO
            int t0=3000, t1=3000, t2=3000, t3=3000;
            vLimiter(prev_theta0, prev_theta1 ,prev_theta2, prev_theta3,
                     theta0_servo, theta1_servo, theta2_servo, theta3_servo,
                     &t0, &t1, &t2, &t3);


            sendServoMoveCommand(0x01, theta0_servo, 500);
            sendServoMoveCommand(0x02, theta1_servo, 500);
            sendServoMoveCommand(0x03, theta2_servo, 500);
            sendServoMoveCommand(0x04, theta3_servo, 500);
            clawServo.write(theta4_servo);

            prev_goal_x=goal_x;
            prev_goal_y=goal_y;
            prev_goal_z=goal_z;
            prev_theta0=theta0_servo;
            prev_theta1=theta1_servo;
            prev_theta2=theta2_servo;
            prev_theta3=theta3_servo;
            
          }
        }

  }
