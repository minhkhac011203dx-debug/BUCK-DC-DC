#include <TimerOne.h>

#define PWM_PIN 9        
#define VOLTAGE_SENSOR_PIN A0 
#define VIN 12.0        

const long PWM_PERIOD_US = 10; 
const int MAX_DUTY_VALUE = 1023;

double Kp = 5; 
double Ki = 3; 
double Kd = 0.0;

double Vout_Target = 5.0;    
double Vout_Measured = 0;   
double Duty_Percent = 0;      

// PID
double error = 0;           
double lastError = 0;        
double integral = 0;          
double derivative = 0;        
unsigned long lastTime = 0;   

int currentDutyValue = 0;
float currentDutyPercent = 0.0;

const float VOLTAGE_DIVIDER_FACTOR = 1.33; 

// ===============================================
// ĐỌC ĐIỆN ÁP ĐẦU RA
// ===============================================
float readOutputVoltage() {
    int adcValue = analogRead(VOLTAGE_SENSOR_PIN);
    float V_adc = (float)adcValue * (5.0 / 1023.0); 
    // Áp dụng hệ số chia áp để ra Vout thực tế
    return V_adc * VOLTAGE_DIVIDER_FACTOR-0.6;
}

// ===============================================
// ĐIỀU KHIỂN PID 
// ===============================================
void computePID() {
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime) / 1000.0; 
    error = Vout_Target - Vout_Measured;

    // Integral = Integral_trước + error * dt
    integral += (error * timeChange);
    
    double integralLimit = 50.0 / Ki; 
    if (integral > integralLimit) integral = integralLimit;
    else if (integral < -integralLimit) integral = -integralLimit;
    // Derivative = (error - lastError) / dt
    if (timeChange > 0) {
        derivative = (error - lastError) / timeChange;
    } else {
        derivative = 0;
    }
    // Output = Kp*P + Ki*I + Kd*D
    Duty_Percent = Kp * error + Ki * integral + Kd * derivative;
    if (Duty_Percent > 100.0) Duty_Percent = 100.0;
    else if (Duty_Percent < 0.0) Duty_Percent = 0.0;
    lastError = error;
    lastTime = now;
}

// ===============================================
// SET DUTY CYCLE
// ===============================================
void setDutyCycle(double percent) {
    if (percent < 0.0) percent = 0.0;
    if (percent > 100.0) percent = 100.0;
    currentDutyPercent = (float)percent;
    currentDutyValue = (int)(currentDutyPercent / 100.0 * MAX_DUTY_VALUE);
    Timer1.setPwmDuty(PWM_PIN, currentDutyValue);
}

// ===============================================
// SETUP: 
// ===============================================
void setup() {
    Serial.begin(115200);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(VOLTAGE_SENSOR_PIN, INPUT);
    
    Serial.println("--- CLOSED-LOOP BUCK CONTROL (MANUAL PID 100kHz) ---");
    Serial.print("Initial Target Vout (Setpoint): ");
    Serial.print(Vout_Target, 1);
    Serial.println(" V");
    Serial.println("Enter new Target Vout (0.0 - 12.0) and press Enter:");
    Timer1.initialize(PWM_PERIOD_US);
    Timer1.pwm(PWM_PIN, 0); 
    lastTime = millis();
}

// ===============================================
// LOOP: 
// ===============================================
void loop() {
    Vout_Measured = readOutputVoltage();
    computePID(); 
    setDutyCycle(Duty_Percent);

    if (Serial.available() > 0) {
        float inputTarget = Serial.parseFloat();
        
        while (Serial.available()) {
            Serial.read();
        }

        if (inputTarget < 0.0) inputTarget = 0.0;
        if (inputTarget > VIN) inputTarget = VIN; 

        Vout_Target = (double)inputTarget;
        
        Serial.print("\n>>> New Target Vout (Setpoint) set to: ");
        Serial.print(Vout_Target, 1);
        Serial.println(" V");
    }

    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 2000) {
        lastPrintTime = millis();
        Serial.print("Target: ");
        Serial.print(Vout_Target, 1);
        Serial.print(" V | Measured: ");
        Serial.print(Vout_Measured, 2);
        Serial.print(" V | Error: ");
        Serial.print(Vout_Target - Vout_Measured, 2);
        Serial.print(" V | Duty: ");
        Serial.print(currentDutyPercent, 1);
        Serial.println(" %");
    }
}
