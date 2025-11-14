#include <iostream>  // 標準入出力
#include <fstream>   // ファイル読み書き
#include <fcntl.h>   // ファイル制御
#include <termios.h> // シリアル通信，端末の入出力設定
#include <unistd.h>  // UNIXの基本API
#include <string>    // 文字列 ex)std::string
#include <cstring>   // メモリ操作
#include <algorithm> // アルゴリズム ex)std::remove

#pragma pack(1)
struct Packet {
    uint32_t node_id;
    float temp_data;
    float humi_data;
};
#pragma pack()

int main() {
    int fd;
    unsigned char buffer[256];
    ssize_t byteRead;
    uint8_t check;
    bool receiving = false;
    int index = 0;

    Packet packet;

    fd = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Error opening serial port" << std::endl;
        return 1;
    }

    // Configure serial port settings
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;                         // No parity
    options.c_cflag &= ~CSTOPB;                         // 1 stop bit
    options.c_cflag &= ~CSIZE;                          // Clear current data size setting
    options.c_cflag |= CS8;                             // 8 data bits
    options.c_cflag &= ~CRTSCTS;                        // No hardware flow control
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
    options.c_oflag &= ~OPOST;                          // Raw output binary
    tcsetattr(fd, TCSANOW, &options);
    std::cout << "Packet correct size : " << sizeof(Packet) << std::endl;
    std::cout << "Serial port configured. Listening for data..." << std::endl;

    while (true) {
        // Read data from serial port
        ssize_t n = read(fd, &check, 1);
        if (n < 0) {
            // std::cerr << "read error" << std::endl;
            continue;
        }

        if (!receiving && check == 'H') {
            tcflush(fd, TCIFLUSH);
            std::cout << "receive start" << std::endl;
            receiving = true;
            index = 0;
            continue;
        }

        if (receiving && check == 'F') {
            std::cout << "receive stop" << std::endl;
            if (index == sizeof(Packet)) {
                memcpy(&packet, buffer, sizeof(packet));

                std::cout << "✅ Received Packet" << std::endl;
                std::cout << "Node ID: 0x" << std::hex << packet.node_id << std::dec << std::endl;
                std::cout << "Temp: " << packet.temp_data << " C, Humi: " << packet.humi_data << " %" << std::endl;

                // Write decoded data to file
                float temperature = packet.temp_data;
                float humidity = packet.humi_data;
                std::cout << "Temperature: " << temperature << " C, Humidity: " << humidity << " %" << std::endl;
                static int fileMax = 10; // Max number of files to save
                // Write temperature data to file
                static int fileCount_temp = 0;
                fileCount_temp %= fileMax;                                                            // Rotate file count
                std::string fileName_temp = "temp_data_" + std::to_string(fileCount_temp++) + ".txt"; // ex) temp_data_0.json, temp_data_1.json, ...
                std::ofstream fout_temp(fileName_temp, std::ios::binary);
                fout_temp << temperature;
                fout_temp.close();
                // Write temperature data to file
                static int fileCount_humi = 0;
                fileCount_humi %= fileMax;                                                            // Rotate file count
                std::string fileName_humi = "humi_data_" + std::to_string(fileCount_humi++) + ".txt"; // ex) humi_data_0.json, humi_data_1.json, ...
                std::ofstream fout_humi(fileName_humi, std::ios::binary);
                fout_humi << humidity;
                fout_humi.close();
                // register Cefore temperature file
                std::string cefName_temp = "ccnx:/lora/data/CH1/temp";
                std::string cmd_temp = "cefputfile " + cefName_temp + " -f " + fileName_temp;
                system(cmd_temp.c_str());
                std::cout << "Registered to Cefore temperature file: " << cefName_temp << " " << fileName_temp << std::endl;
                // register Cefore humidity file
                std::string cefName_humi = "ccnx:/lora/data/CH1/humi";
                std::string cmd_humi = "cefputfile " + cefName_humi + " -f " + fileName_humi;
                system(cmd_humi.c_str());
                std::cout << "Registered to Cefore humidity file: " << cefName_humi << " " << fileName_humi << std::endl;
            } else {
                std::cout << "size mismatch got : " << index << std::endl;
            }
            receiving = false;
            continue;
        }

        if (receiving) {
            if (index < sizeof(Packet)) {
                buffer[index++] = check;
            } else {
                receiving = false;
                index = 0;
                std::cerr << "buffer Overflow!" << std::endl;
            }
        }

        usleep(100000); // Sleep for 0.1s
    }

    close(fd);
    return 0;
}