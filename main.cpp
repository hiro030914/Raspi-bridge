#include <iostream>              // For standard I/O operations
#include <fstream>               // For file I/O operations
#include <fcntl.h>               // For file control options
#include <termios.h>             // For terminal I/O
#include <unistd.h>              // For UNIX API
#include <string>                // For std::string
#include <mbedtls/base64.h>      // For Base64 encoding/decoding

int main() {
  int fd;
  unsigned char buffer[256];
  ssize_t byteRead;

fd = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY | O_NDELAY);
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
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;   // Clear current data size setting
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cflag &= ~CRTSCTS; // No hardware flow control
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    options.c_oflag &= ~OPOST; // Raw output binary
    tcsetattr(fd, TCSANOW, &options);

    while(true) {
    // Read data from serial port
    byteRead = read(fd, buffer, sizeof(buffer));
      if (byteRead > 0) {
        std::cout << "Read " << byteRead << " bytes from serial port" << std::endl;

     // Base64 decode the data
     unsigned char decoded[512];
     size_t decoded_len = 0;

     int ret = mbedtls_base64_decode(decoded, sizeof(decoded), &decoded_len, reinterpret_cast<const unsigned char*>(buffer), byteRead);
       if (ret != 0) {
         std::cerr << "Base64 decoding failed" << std::endl;
         continue;
       }

     // Write decoded data to file
     static int fileCount = 0;
       std::string fileName = "data_" + std::to_string(fileCount++) + ".bin";  // ex) data_0.bin, data_1.bin, ...
       std::ofstream fout(fileName, std::ios::binary | std::ios::app);
       fout.write(reinterpret_cast<char*>(decoded), decoded_len);
       fout.close();

       std::cout << "Saved decoded data to " << fileName << std::endl;

      // register Cefore
      std::string cefName = "ccnx:/lora/data/" + fileName;
      std::string cmd = "cefputfile " + cefName + " " + fileName;
      system(cmd.c_str());
      std::cout << "Registered to Cefore: " << cefName << std::endl;
    } else if (byteRead <= 0) {
        std::cerr << "Error reading from serial port" << std::endl;
        continue;
     }

     usleep(100000); // Sleep for 0.1s
    }
  
  close(fd);
  return 0;

}