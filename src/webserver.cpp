#include "Robot.hpp"
WebServer server(80);

Position positions = {0.0, 0.0, 0.0};

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
    positions.x = server.arg("input1").toFloat();
    positions.y = server.arg("input2").toFloat();
    positions.z = server.arg("input3").toFloat();

    Serial.println("Received Positions:");
    server.send(200, "text/html", "Submission received. <a href=\"/\">Return to form</a>");
    movetoPos(current_position, positions, motor1, motor2);
}