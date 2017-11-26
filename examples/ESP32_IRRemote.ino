#include <esp32_rmt.h>
const int RECV_PIN = 19; //Receive GPIO
ESP32_RMT remote(RECV_PIN);

void setup()
{
	Serial.begin(115200);
	xTaskCreate(irRecieveTask,"irRecieveTask", 2048, NULL, 10, NULL);
}

void loop()
{
	if (remote.result>0) {
		Serial.println(remote.result,HEX);
		remote.result = 0;
	}
}

void irRecieveTask(void *pvParameters)
{
	while(1) {
		remote.irRecieve();
	}
}