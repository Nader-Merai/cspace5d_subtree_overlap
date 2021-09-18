#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

int main() {
    std::fstream fs[10];
    std::string buf;
    std::vector<double> time;
    std::vector<double> cost;
    std::vector<double> samples;
    double time_avg(0), cost_avg(0), samples_avg(0);

    int j = 0;
    for (int i = 0; i < 10; ++i) {
        j++;
        fs[i].open(std::to_string(i) + ".log");
        int a, b;
        while( getline(fs[i], buf) ){
            if(std::string::npos != buf.find("Time: ")){
                auto m = buf.find("s");
                time.push_back(atof(buf.substr(6,m-6).c_str()));

                getline(fs[i], buf);
                getline(fs[i], buf);
                if(std::string::npos != buf.find("solution cost: ")){
                    auto n = buf.find(":");
                    cost.push_back(atof(buf.substr(n+1).c_str()));
                }else{
                    time.pop_back();
                    break;
                }

                getline(fs[i], buf);
                if(std::string::npos != buf.find("Number of samples: ")){
                    auto n = buf.find(":");
                    samples.push_back(atof(buf.substr(n+1).c_str()));
                } else {
                    time.pop_back();
                    break;
                }
            }
        }
    }

    for(int i = 0; i < time.size(); ++i ){
        time_avg += time[i];
        cost_avg += cost[i];
        samples_avg += samples[i];
    }
    
    time_avg = time_avg/time.size();
    cost_avg = cost_avg/cost.size();
    samples_avg = samples_avg/samples.size();
    double rate = time.size() / 10.0;

    std::cout << "time: " << time_avg << " cost: " << cost_avg << " samples: " << samples_avg << 
         " rate:" << rate << std::endl;

    for (int i = 0; i < 10; ++i) {
        fs[i].close();
    }

    return 0;
}
