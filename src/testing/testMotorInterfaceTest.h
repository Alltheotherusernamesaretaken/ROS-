#include<MotorInterfaceTest.h>

MotorInterfaceTest testmotor;

void setup(){
    // Serial Debug output
    Serial.begin(921600);
    Serial.println("Beginning MotorInterfaceTest");
}

void loop(){
    Serial.println("test update");
    Serial.println(0==testmotor.update());
    Serial.println("test setPos");
    Serial.println(0==testmotor.setPos(1.0));
    Serial.println("test setVel");
    Serial.println(0==testmotor.setVel(1.0));
    double stpt = 0;
    Serial.println("test getPos");
    Serial.println(0==testmotor.getPos(&stpt));
    Serial.println(1==stpt);
    stpt=0;
    Serial.println("test getVel");
    Serial.println(0==testmotor.getVel(&stpt));
    Serial.println(1==stpt);
    stpt=0;

    delay(4000);

}