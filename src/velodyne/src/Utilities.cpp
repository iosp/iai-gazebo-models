/**
* Utilities.cpp
* Author: Binyamin Appelbaum
* Date: 3.12.17
*/

#include "Utilities.h"
#include <sstream>
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp> // boost::posix_time::ptime


bool Utilities::MakeDirectory(const std::string& dirName, mode_t mode) {
    return (mkdir(dirName.c_str(), mode) == 0);
}

double Utilities::dmod(double num, double mod) {
    while (num > mod) {
        num -= mod;
    }
    return num;
}

std::string Utilities::GetFormattedTime(const std::string& format) {
    boost::posix_time::time_facet * facet = new boost::posix_time::time_facet(format.c_str());
    std::ostringstream stream;
    stream.imbue(std::locale(stream.getloc(), facet));
    stream << boost::posix_time::second_clock::local_time();
    return stream.str();
}

void Utilities::PrintToFile(const std::string& fileName, const std::string& text) {
    std::ofstream file(fileName, std::ios_base::app);
    file << text;
    file.close();
}