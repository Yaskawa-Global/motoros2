#include <iostream>
#include <string>
#include <winsock2.h> // For Winsock functions
#include <ws2tcpip.h> // For inet_pton

// Link with Ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")

// Function to handle errors
void error(const char* msg) {
    std::cerr << msg << ": " << WSAGetLastError() << std::endl;
    WSACleanup(); // Clean up Winsock
    exit(1);
}

int main() {
    WSADATA wsaData;

    // 1. Initialize Winsock
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        std::cerr << "WSAStartup failed: " << iResult << std::endl;
        return 1;
    }

    // 2. Create a socket
    SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET) {
        error("Error at socket()");
    }

    // Set TCP_NODELAY option
    char optval = 1; // Use char for boolean options in Winsock
    iResult = setsockopt(clientSocket, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));
    if (iResult == SOCKET_ERROR) {
        error("Error setting TCP_NODELAY option");
    }
    std::cout << "TCP_NODELAY set on socket." << std::endl;

    // 3. Specify server address and port
    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(50245); // Port number 50245

    // You'll need to replace "127.0.0.1" with the actual IP address of your server
    if (inet_pton(AF_INET, "192.168.1.31", &serverAddress.sin_addr) <= 0) {
        error("Invalid address/ Address not supported");
    }

    // 4. Connect to the server
    iResult = connect(clientSocket, (SOCKADDR*)&serverAddress, sizeof(serverAddress));
    if (iResult == SOCKET_ERROR) {
        closesocket(clientSocket);
        clientSocket = INVALID_SOCKET;
        error("Unable to connect to server");
    }

    std::cout << "Connected to server on port 50245." << std::endl;

    // 5. Send and receive data (example)
    std::string message = "Hello!";
    iResult = send(clientSocket, message.c_str(), (int)message.length(), 0);
    if (iResult == SOCKET_ERROR) {
        error("Error sending message");
    }
    //std::cout << "Sent: " << message << std::endl;

    char buffer[1024];
    iResult = recv(clientSocket, buffer, sizeof(buffer) - 1, 0); // Leave space for null terminator
    if (iResult > 0) {
        buffer[iResult] = '\0'; // Null-terminate the received data
        //std::cout << "Received: " << buffer << std::endl;
    }
    else if (iResult == 0) {
        std::cout << "Server closed the connection." << std::endl;
    }
    else {
        error("Error receiving message");
    }

    // 6. Close the socket
    iResult = closesocket(clientSocket);
    if (iResult == SOCKET_ERROR) {
        error("Error closing socket");
    }

    // 7. Clean up Winsock
    WSACleanup();
    std::cout << "Connection closed." << std::endl;

    return 0;
}
