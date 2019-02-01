#include "node.cpp"
#include "Stack.cpp"
#include "Queue.cpp"

#include "Core.cpp"
#include "PathPlanning.cpp"
#include "Tests.cpp"

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing ...");

  Core::InitRobot();

  // Testing Framework
  Core::TestPathPlanning();
}

void loop() {
  // put your main code here, to run repeatedly:
  double num = 15.0;
  float num2 = 33.0;

  Stack<int> stack;
  // Serial.println(stack.top_element_index);

  Serial.println(sqrt(num2), 10);

  delay(500);
}
