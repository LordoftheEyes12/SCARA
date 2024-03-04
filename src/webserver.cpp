#include "Robot.hpp"


Position positions_web = {0.0, 0.0, 0.0};

void startWebServerTask(void *pvParameters)
{
    server.on("/", HTTP_GET, handleRoot);
    server.on("/get", HTTP_GET, handleFormSubmit);
    server.begin();
    Serial.println("HTTP server started");

    for (;;)
    {
        server.handleClient();
    }
}

void handleRoot()
{
    String html = "<!DOCTYPE html><html><body>"
                  "<h1>ESP32 Web Server</h1>"
                  "<form action=\"/get\">"
                  "Input 1:<br><input type=\"text\" name=\"input1\"><br>"
                  "Input 2:<br><input type=\"text\" name=\"input2\"><br>"
                  "Input 3:<br><input type=\"text\" name=\"input3\"><br><br>"
                  "<input type=\"submit\" value=\"Submit\">"
                  "</form></body></html>";

    server.send(200, "text/html", html);
}

void handleFormSubmit()
{
    positions_web.x = server.arg("input1").toFloat();
    positions_web.y = server.arg("input2").toFloat();
    positions_web.z = server.arg("input3").toFloat();

    Serial.println("Received positions_web:");
    server.send(200, "text/html", "Submission received. <a href=\"/\">Return to form</a>");
    movetoPos(current_position, positions_web, motor1, motor2);
}