#ifndef __read_csv_h__
#define __read_csv_h__

#include <string>
#include <fstream>
#include <vector>
#include <utility>
#include <sstream>


std::vector<std::pair<std::string, std::vector<double> > > read_csv(const std::string &filename){
    std::vector<std::pair<std::string, std::vector<double> > > result;
    std::ifstream infile(filename);
    if (!infile.is_open()) {
    	std::cerr << "The file '" << filename << "' could not be opened!" << std::endl;
    	return {};
  	}

    std::string line, var;
    double val;
    if (infile.good()) {
        std::getline(infile, line);
        std::stringstream ss(line);
        while (std::getline(ss, var, ',')) {
            result.push_back({var, {}});
        }
    }
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        for (int i = 0; i < (int)result.size(); i ++ ) {
        	ss > > val;
        	result[i].second.push_back(val);
        	if (ss.peek() == ',') ss.ignore();
        }
    }
    infile.close();

    return result;
}

#endif