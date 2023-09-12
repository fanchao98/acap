#pragma once
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <map>
#include <array>
#include <string>
class Config {
public:
    float get(const char* parameter);
    const char* getString(const char* parameter);

    void loadConfig(const char* filename);

private:
    // a map holding configuration values read from config.ini
    std::map<std::string, float> config;
    std::map<std::string, std::string> configString;
    const char* configFileName;

};

#endif //__CONFIG_H__
