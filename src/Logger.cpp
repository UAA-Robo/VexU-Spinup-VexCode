#include <Logger.h>

Logger::Logger(Hardware* hardware, std::string fileName, std::vector<std::string> cols) {
    hw = hardware;
    
    if (hw->brain.SDcard.isInserted()){
        std::cout << "Initialized Logger Here" << std::endl;
        dataLog.open(fileName, std::ofstream::out | std::ofstream::trunc);
        
        //Label column names
        dataLog << "EpochTime, ";
        for (int i = 0; i < cols.size() - 1; i++) {
            dataLog << cols.at(i) << ", ";
        }
        dataLog << cols.at(cols.size() - 1) << "\n";
    }
    
}
Logger::~Logger() {
    std::cout << "Closing file." << std::endl;
    dataLog.close();
}

void Logger::addData(std::vector<double> row) {
    if (hw->brain.SDcard.isInserted()){
        std::cout << "Adding data" << std::endl;
        time_t now = time(0);
        std::string date_time = ctime(&now);
        dataLog << date_time << ", ";

        for (int i = 0; i < row.size() - 1; i++) {
            dataLog << row.at(i) << ", ";
        }
        dataLog << row.at(row.size() - 1) << "\n";

    }
}