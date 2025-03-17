Project Description
This project provides an ESP32-based bridge that creates a Wi-Fi link between a UD3 controller and a computer via UDP MIN frame recognition. The code sets up a Wi-Fi access point named TESLATERM (password: 12345678), listens for UDP packets on port 1337, and uses the static IP address 192.168.4.1. The packets received from the computer are forwarded to the UD3 controller and vice versa, enabling efficient wireless communication.




Key Features

Automatically configures an ESP32-based Wi-Fi access point (SSID: TESLATERM, password: 12345678).
Assigns the static IP address 192.168.4.1 to the ESP32.
Listens for and forwards UDP traffic on port 1337.
Facilitates bidirectional data transmission, making it easy to communicate with the UD3 controller in real time.



How to Use

Flash the ESP32 with the code from this repository.
Connect to the “TESLATERM” Wi-Fi network using the password “12345678.”
Send or receive UDP packets on port 1337 at IP address 192.168.4.1.
The ESP32 bridge will relay data to and from the UD3 controller via its serial interface, enabling real-time data exchange.
