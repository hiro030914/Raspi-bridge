#include <iostream>              // For standard I/O operations
#include <fstream>               // For file I/O operations
#include <fcntl.h>               // For file control options
#include <termios.h>             // For terminal I/O
#include <unistd.h>              // For UNIX API
#include <string>                // For std::string
#include <cstring>               // For memset and memcpy
#include <algorithm>             // For std::remove
#include <mbedtls/base64.h>      // For Base64 encoding/decoding

using namespace std;


int main() {
  int fd;
  unsigned char buffer[512];
  ssize_t byteRead;

fd = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    cerr << "Error opening serial port" << endl;
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

    cout << "Serial port configured. Listening for data..." << endl;
    string line;

    while(true) {
    // Read data from serial port
    byteRead = read(fd, buffer, sizeof(buffer));
      if (byteRead > 0) {
        if(byteRead >= sizeof(buffer)) {
          cerr << "Buffer overflow detected" << endl;
          continue;
        } else {
          buffer[byteRead] = '\0'; // Null-terminate the buffer
          line += string((char*)buffer);
        }


        
        cout << "Read " << byteRead << " bytes from serial port" << endl;

      // Check for newline character to determine end of message
      if (line.find('\n') == string::npos) {
        continue; // Wait for more data
      }
      // Process complete line
      string completeLine = line.substr(0, line.find('\n'));
      line.erase(0, line.find('\n') + 1); // Remove processed line from buffer
      cout << "Complete line received: " << completeLine << endl;

      //delete spaces and newlines
      completeLine.erase(remove(completeLine.begin(), completeLine.end(), ' '), completeLine.end());
      completeLine.erase(remove(completeLine.begin(), completeLine.end(), '\n'), completeLine.end());
      completeLine.erase(remove(completeLine.begin(), completeLine.end(), '\r'), completeLine.end());

      // trim header 
      size_t firstsep_payroll = completeLine.find('|');
      size_t secondsep_payroll = completeLine.find('|', firstsep_payroll + 1);
      if (firstsep_payroll == string::npos || secondsep_payroll == string::npos) {
        cerr << "Invalid data format" << endl;
        continue;
      }

      completeLine = completeLine.substr(secondsep_payroll +1);

      cout << "Data to decode: " << completeLine << endl;

      // Copy complete line to buffer for decoding
      byteRead = completeLine.length();
      memcpy(buffer, completeLine.c_str(), byteRead);

     // Base64 decode the data
     unsigned char decoded[512];
     size_t decoded_len = 0;

     int ret = mbedtls_base64_decode(decoded, sizeof(decoded), &decoded_len, (unsigned char*)completeLine.c_str(), byteRead);
       if (ret != 0) {
         cerr << "Base64 decoding failed" << endl;
         continue;
       }
       string decodedStr((char*)decoded, decoded_len);
       cout << "Decoded data: " << decodedStr << endl;


       // separate temperature value from JSON
       size_t firstsep_temp = decodedStr.find(':');
       size_t secondsep_temp = decodedStr.find(",", firstsep_temp +  1);
       if(firstsep_temp == string::npos || secondsep_temp == string::npos) {
           cerr << "Invalid JSON format" << endl;
           continue;
       }
       string temp_value = decodedStr.substr(firstsep_temp + 1, secondsep_temp - firstsep_temp - 1);
       cout << "Extracted temperature value: " << temp_value << endl;

       // separate Humidity value from JSON
       size_t firstsep_humi = decodedStr.find(':', secondsep_temp);
       size_t secondsep_humi = decodedStr.find("}", firstsep_humi +  1);
       if(firstsep_humi == string::npos || secondsep_humi == string::npos) {
           cerr << "Invalid JSON format" << endl;
           continue;
       }
       string humi_value = decodedStr.substr(firstsep_humi + 1, secondsep_humi - firstsep_humi - 1);
       cout << "Extracted humidity value: " << humi_value << endl;

       float temperature = stof(temp_value);
       float humidity = stof(humi_value);
       cout << "Temperature: " << temperature << " C, Humidity: " << humidity << " %" << endl;

        static int fileMax = 10; // Max number of files to save

      // Write temperature data to file
     static int fileCount_temp = 0;
     fileCount_temp %= fileMax; // Rotate file count
       string fileName_temp = "temp_data_" + std::to_string(fileCount_temp++) + ".json";  // ex) temp_data_0.json, temp_data_1.json, ...
       ofstream fout_temp(fileName_temp, std::ios::binary);
       fout_temp << temp_value;
       fout_temp.close();

      // Write temperature data to file
      static int fileCount_humi = 0;
      fileCount_humi %= fileMax; // Rotate file count
       string fileName_humi = "humi_data_" + to_string(fileCount_humi++) + ".json";  // ex) humi_data_0.json, humi_data_1.json, ...
       ofstream fout_humi(fileName_humi, ios::binary);
       fout_humi << humi_value;
       fout_humi.close();

      // register Cefore temperature file
      string cefName_temp = "ccnx:/lora/data/CH1/temp";
      string cmd_temp = "cefputfile " + cefName_temp + " -f " + fileName_temp;
      system(cmd_temp.c_str());
      cout << "Registered to Cefore temperature file: " << cefName_temp << " " << fileName_temp << endl;

      // register Cefore humidity file
      string cefName_humi = "ccnx:/lora/data/CH1/humi";
      string cmd_humi = "cefputfile " + cefName_humi + " -f " + fileName_humi;
      system(cmd_humi.c_str());
      cout << "Registered to Cefore humidity file: " << cefName_humi << " " << fileName_humi << endl;
    } else if (byteRead <= 0) {
       // cerr << "Error reading from serial port" << endl;
        continue;
     }

     usleep(100000); // Sleep for 0.1s
    }
  
  close(fd);
  return 0;
}