
#include "Arduino.h"
#include "application.h"

Application app;

void setup(){
	app.setup();
}

void loop(){
	app.loop();
}
