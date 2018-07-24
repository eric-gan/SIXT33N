/*
 * open_loop.ino
 *
 * Eric Gan, Shloak Jain, Arjun Mishra
 */

#define LEFT_MOTOR                  P2_0
#define RIGHT_MOTOR                 P1_5

#define RUN_TIME                    (15*1000)

unsigned long end_time = 0;

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*---------------------------*/
float theta_left = 0.5704;
float theta_right = 0.5996;
float beta_left = 32.04;
float beta_right = 25.31;
float v_star = 29.1;


// PWM inputs to jolt the car straight
int left_jolt = 240;
int right_jolt = 200;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/

float driveStraight_left_OL(float v_star) {
  return (v_star + beta_left) / theta_left;
}

float driveStraight_right_OL(float v_star) {
  return (v_star + beta_right) / theta_right;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


void setup(void) {
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  write_pwm(0, 0); // Turn off motors
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker(); // Blink lights to indicate car is running
  write_pwm(left_jolt, right_jolt); // Jolt motors for 200ms
  delay(200);

  /*---------------------------*/
  /*      CODE BLOCK CON0      */
  /*---------------------------*/

  // Attempt to drive straight using open loop control
  // Compute the PWM input required for each wheel based on v_star
  int left_cur_pwm = driveStraight_left_OL(v_star);
  int right_cur_pwm = driveStraight_right_OL(v_star);
  write_pwm(left_cur_pwm, right_cur_pwm);

  /*---------------------------*/
  /*---------------------------*/
  /*---------------------------*/

  end_time = millis() + RUN_TIME;
}

void loop(void) {
  if (end_time <= millis()) {
    write_pwm(0, 0);
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}
