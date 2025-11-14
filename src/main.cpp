#include <iostream>  // 標準入出力
#include <fstream>   // ファイル読み書き
#include <fcntl.h>   // ファイル制御
#include <termios.h> // シリアル通信，端末の入出力設定
#include <unistd.h>  // UNIXの基本API
#include <string>    // 文字列 ex)std::string
#include <cstring>   // メモリ操作
#include <algorithm> // アルゴリズム ex)std::remove

// packet構造体はセンサノードが送信するデータ型
// pragma pack(1)によりパディングを削除
#pragma pack(1)
struct Packet {
    uint32_t node_id;  // センサノード識別子
    float temp_data;   // 温度データ
    float humi_data;   // 湿度データ
};
#pragma pack()

int main() {
    int fd;                       // ポート開放確認用
    unsigned char buffer[256];    // バッファサイズ
    uint8_t check;                // 受信パケット，ヘッダ・フッタ確認
    bool receiving = false;       // 状態フラグ(true:アイドル，false:受信待機)
    int index = 0;                // 受信バッファインデックス

    Packet packet;

    // シリアルポートの初期化 (O_RDONLY:読み込み専用，O_NOCTTY:制御端末としない)
    fd = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Error opening serial port" << std::endl;
        return 1;
    }

    // シリアルポートの設定のためtermios構造体で宣言
    struct termios options;

    // 現在のシリアルポートの設定取得
    tcgetattr(fd, &options);

    // 通信速度設定
    cfsetispeed(&options, B115200);                    // 受信側ボーレート:115200
    cfsetospeed(&options, B115200);                    // 送信側ボーレート:115200

    // 制御フラグ設定
    options.c_cflag |= (CLOCAL | CREAD);                // ローカル接続/受信有効
    options.c_cflag &= ~PARENB;                         // パリティ無
    options.c_cflag &= ~CSTOPB;                         // 1 stop bit
    options.c_cflag &= ~CSIZE;                          // ビットマスク初期化
    options.c_cflag |= CS8;                             // データビット 8bit
    options.c_cflag &= ~CRTSCTS;                        // ハードウェアフロー制御無

    // ローカルフラグ設定
    // 制御端末モード無効化(非カノニカルモード，エコー無，シグナル無)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 

    // 入力フラグ設定
    // ソフトウェアフロー制御なし(XON/XOFF無効)
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 出力フラグ設定
    // 出力データをバイナリのまま
    options.c_oflag &= ~OPOST;

    // 変更を反映
    tcsetattr(fd, TCSANOW, &options);
    std::cout << "Packet correct size : " << sizeof(Packet) << std::endl;
    std::cout << "Serial port configured. Listening for data..." << std::endl;

    // デバッグ用
    std::cout << "Packet correct size : " << sizeof(Packet) << std::endl;

    // 受信待機
    std::cout << "Serial port configured. Listening for data..." << std::endl;

    while (true) {
        // シリアルポートからデータを1byteずつ受信
        ssize_t n = read(fd, &check, 1);

        // 受信失敗の場合
        if (n < 0) {
            // std::cerr << "read error" << std::endl;
            continue;
        }

        // ヘッダパケット受信時処理
        if (!receiving && check == 'H') {
            tcflush(fd, TCIFLUSH);        // 受信後読みとられなかったデータ破棄
            std::cout << "receive start" << std::endl;
            receiving = true;
            index = 0;
            continue;
        }

        // ヘッダ受信後Packetサイズ(12bit)までバッファに保管
        if (receiving && check != 'F') {
            if (index < sizeof(Packet)) {
                buffer[index++] = check;
            } else {
                receiving = false;
                index = 0;
                std::cerr << "buffer Overflow!" << std::endl;
            }
        }

        // フッタパケット受信時処理
        if (receiving && check == 'F') {
            std::cout << "receive stop" << std::endl;

            // 受信データとPacketのサイズ(12bit)と一致時の処理
            if (index == sizeof(Packet)) {
                memcpy(&packet, buffer, sizeof(packet));    // 受信パケットを構造体にコピー

                // デバッグ用
                std::cout << "✅ Received Packet" << std::endl;
                std::cout << "Node ID: 0x" << std::hex << packet.node_id << std::dec << std::endl;
                std::cout << "Temp: " << packet.temp_data << " C, Humi: " << packet.humi_data << " %" << std::endl;

                // 受信したセンサデータを書き込み
                float temperature = packet.temp_data;
                float humidity = packet.humi_data;

                // デバッグ用
                std::cout << "Temperature: " << temperature << " C, Humidity: " << humidity << " %" << std::endl;

                static int fileMax = 10;            // 作成ファイルの循環最大数

                // 温度データファイル書き込み処理
                static int fileCount_temp = 0;
                fileCount_temp %= fileMax;          // 作成ファイルの循環
                std::string fileName_temp = "temp_data_" + std::to_string(fileCount_temp++) + ".txt"; // ex) temp_data_0.txt, temp_data_1.txt, ...

                // ファイル作成処理(バイナリモード)
                std::ofstream fout_temp(fileName_temp, std::ios::binary);
                fout_temp << temperature;  // 文字列で書き込み処理→バイナリで書き込みに変更予定
                fout_temp.close();

                // 湿度データファイル書き込み処理
                static int fileCount_humi = 0;
                fileCount_humi %= fileMax;                                                            // Rotate file count
                std::string fileName_humi = "humi_data_" + std::to_string(fileCount_humi++) + ".txt"; // ex) humi_data_0.json, humi_data_1.json, ...
                std::ofstream fout_humi(fileName_humi, std::ios::binary);
                fout_humi << humidity;
                fout_humi.close();

                // 作成ファイル(温度)をCeforeに登録
                std::string cefName_temp = "ccnx:/lora/data/CH1/temp";                         // uri作成
                std::string cmd_temp = "cefputfile " + cefName_temp + " -f " + fileName_temp;  // 登録コマンド作成

                system(cmd_temp.c_str());  // コマンド実行

                // 登録完了通知
                std::cout << "Registered to Cefore temperature file: " << cefName_temp << " " << fileName_temp << std::endl;

                // 作成ファイル(温度)をCeforeに登録
                std::string cefName_humi = "ccnx:/lora/data/CH1/humi";                         // uri作成
                std::string cmd_humi = "cefputfile " + cefName_humi + " -f " + fileName_humi;  // 登録コマンド作成

                system(cmd_humi.c_str());  // コマンド実行
                
                // 登録完了通知
                std::cout << "Registered to Cefore humidity file: " << cefName_humi << " " << fileName_humi << std::endl;

            } else {
                std::cout << "size mismatch got : " << index << std::endl;  // サイズ不一致でエラー
            }
            receiving = false;  // フッタ受信により受信待機状態
            continue;
        }
        usleep(100000); // スリープ 0.1s
    }

    close(fd);
    return 0;
}