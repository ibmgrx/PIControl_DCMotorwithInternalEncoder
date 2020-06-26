/*===========================================
PRAKTIKUM KONTROL DIGITAL 2020 SIMULASI GAESS
===========================================*/

//Variabel Kontrol PID
double kp = 1;
double ki = 0;
double kd = 0;//DIBUAT 0 KARENA HANYA PERCOBAAN KONTROL PI
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

//
int motorPin = 9;
int pwmSignal = 0;
int interruptPin = 2;

//Motor encoder Counter
volatile int count = 0;
//RPM
float speedRPM = 0;
int signal = 10;

//HITUNG PID
double computePID(double inp){     
  
        error = setPoint - inp;                          
        cumError += error;                
        rateError = (error - lastError);   
 
        double out = kp*error + ki*cumError + kd*rateError;                
        lastError = error;                                                    
 
        return out;                                       
}

//HITUNG RPM
void encoderPulse()
{
  count++;
} 


void setup()
{
  Serial.begin(9600);
  pinMode(6, OUTPUT);
  
  //SETTING INTERRUPT
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), encoderPulse, RISING);
  sei();
}

void loop()
{
  //ATUR SETPOINT
  setPoint = 150; //100+(NO.Kel x 10)
  //DRIVE MOTOR
  analogWrite(motorPin,signal);
  
  //HITUNG RPM
  count = 0;
  delay(200);
  speedRPM = count * 6.25;
  input = speedRPM;
  
  //HITUNG PID
  output = computePID(input);
  delay(100);
  //UBAH OUTPUT PID JADI NILAI PWM
  pwmSignal = map((int)output,0,355,0,255);
  delay(50);
  signal = pwmSignal;
  Serial.println(speedRPM);
}

