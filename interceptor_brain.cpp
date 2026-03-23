#include <iostream>
#include <cmath>
#include <chrono>

// --- CROSS-PLATFORM NETWORKING HEADERS ---
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #define SOCKET int
    #define INVALID_SOCKET (-1)
    #define SOCKET_ERROR (-1)
    #define closesocket close
#endif

// Include the standard ArduPilot MAVLink dialect
#include "mavlink/ardupilotmega/mavlink.h"

#define R 6378137.0 
#define N_GAIN 4.0

double get_time_seconds() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    return duration.count() / 1000000.0;
}

int main() {
    std::cout << "Initializing 3D C++ Interceptor Brain (Cross-Platform)..." << std::endl;

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed." << std::endl;
        return 1;
    }
#endif

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Error creating socket." << std::endl;
        return 1;
    }

    struct sockaddr_in locAddr;
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(14550);

    if (bind(sock, (struct sockaddr *)&locAddr, sizeof(locAddr)) == SOCKET_ERROR) {
        std::cerr << "Error: UDP Bind failed. Is another script using port 14550?" << std::endl;
        closesocket(sock);
        return 1;
    }

#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);
#else
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif

    double int_lat = 19.0100;
    double int_lon = 73.0500;
    double int_alt = 0.0;
    double int_speed = 35.0;
    double int_heading_az = 45.0;
    double int_heading_el = 20.0;

    double previous_los_az = 0.0;
    double previous_los_el = 0.0;
    bool has_previous_los = false;

    double last_time = get_time_seconds();

    std::cout << "\n--- LAUNCHING 3D INTERCEPTOR (C++) ---" << std::endl;

    char buffer[2048];
    mavlink_message_t msg;
    mavlink_status_t status;

    while (true) {
        int recsize = recv(sock, buffer, sizeof(buffer), 0);
        
        if (recsize > 0) {
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                        mavlink_global_position_int_t pos;
                        mavlink_msg_global_position_int_decode(&msg, &pos);

                        double current_time = get_time_seconds();
                        double dt = current_time - last_time;
                        if (dt <= 0) continue;
                        last_time = current_time;

                        double target_lat = pos.lat / 1e7;
                        double target_lon = pos.lon / 1e7;
                        double target_alt = pos.alt / 1000.0;

                        double dy_north = (target_lat - int_lat) * (3.14159265 / 180.0) * R;
                        double dx_east = (target_lon - int_lon) * (3.14159265 / 180.0) * R * cos(int_lat * 3.14159265 / 180.0);
                        double dz_up = target_alt - int_alt;

                        double ground_distance = sqrt(dx_east*dx_east + dy_north*dy_north);
                        double slant_distance = sqrt(ground_distance*ground_distance + dz_up*dz_up);

                        if (slant_distance < 15.0) {
                            std::cout << "\n[BOOM] TARGET INTERCEPTED IN 3D! Slant Range: " << slant_distance << "m\n";
                            closesocket(sock);
#ifdef _WIN32
                            WSACleanup();
#endif
                            return 0;
                        }

                        double los_az = atan2(dy_north, dx_east);
                        double los_el = atan2(dz_up, ground_distance);

                        if (!has_previous_los) {
                            previous_los_az = los_az;
                            previous_los_el = los_el;
                            has_previous_los = true;
                        }

                        double los_rate_az = (los_az - previous_los_az) / dt;
                        double los_rate_el = (los_el - previous_los_el) / dt;

                        previous_los_az = los_az;
                        previous_los_el = los_el;

                        double accel_az = N_GAIN * int_speed * los_rate_az;
                        double accel_el = N_GAIN * int_speed * los_rate_el;

                        double turn_rate_az = accel_az / int_speed;
                        double turn_rate_el = accel_el / int_speed;

                        int_heading_az += turn_rate_az * dt * (180.0 / 3.14159265);
                        int_heading_el += turn_rate_el * dt * (180.0 / 3.14159265);

                        if (int_heading_el > 80.0) int_heading_el = 80.0;
                        if (int_heading_el < -80.0) int_heading_el = -80.0;

                        double vz = int_speed * sin(int_heading_el * 3.14159265 / 180.0);
                        double ground_speed = int_speed * cos(int_heading_el * 3.14159265 / 180.0);
                        double vx = ground_speed * cos(int_heading_az * 3.14159265 / 180.0);
                        double vy = ground_speed * sin(int_heading_az * 3.14159265 / 180.0);

                        int_alt += vz * dt;
                        int_lat += (vy * dt / R) * (180.0 / 3.14159265);
                        int_lon += (vx * dt / R) * (180.0 / 3.14159265) / cos(int_lat * 3.14159265 / 180.0);

                        printf("Dist: %.1fm | Alt: %.1fm | Az Cmd: %.1fm/s2 | El Cmd: %.1fm/s2\n", 
                               slant_distance, int_alt, accel_az, accel_el);
                    }
                }
            }
        }
    }
    return 0;
}