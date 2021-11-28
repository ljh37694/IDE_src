#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.1 // EMA filter is disabled

// Servo range
#define _DUTY_MIN 800
#define _DUTY_NEU 1250
#define _DUTY_MAX 1800

// Servo speed control
#define _SERVO_ANGLE 30 // angle b/w DUTY_MAX and DUTY_MIN
#define _SERVO_SPEED 600 // servo speed limit (deg/sec)

// Event periods
#define _INTERVAL_DIST 20 // distance sensor interval (ms)
#define _INTERVAL_SERVO 20 // servo interval (ms)
#define _INTERVAL_SERIAL 100 // serial interval (ms)

// PID parameters
#define _KP 1 // proportional gain *****

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;
// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; // maximum duty difference per interval
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);
  delay(5000);
// initialize global variables
  dist_target = _DIST_TARGET;

  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = false;
  event_servo = false;
  event_serial = false;
  
// move servo to neutral position

// initialize serial port
  Serial.begin(115200);
  
// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) / 180 * _SERVO_SPEED * _INTERVAL_SERVO / 1000;
}

void loop() {
/////////////////////
// Event generator //
/////////////////////

  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  
////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
  // dist_raw = ir_distance();
    dist_raw = ir_distance_filtered();
    dist_ema = _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
    
  // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = error_curr;
    control = _KP * pterm;
    
  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    else if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    
  // keep duty_target value within the range of
  // [_DUTY_MIN, _DUTY_MAX]
  }

  if(event_servo) {
// adjust duty_curr toward duty_target by duty_chg_per_interval
// update servo position
    event_servo = false;
    
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
      
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(pterm);
    //Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    //Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
  
float ir_distance_filtered(void){ // return value unit: mm
  int a = 66;
  int b = 310;
  
  return 100 + 300.0 / (b - a) * (ir_distance() - a);
}
