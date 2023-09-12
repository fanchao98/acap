#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <assert.h>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <array>
#include <math.h>
#include <exception>
#include <fstream>

#include "config.h"

// read a config value
float Config::get(const char* parameter) {
    // ensure sure the value was set by the config file
    assert(this->config.count(parameter) == 1);

    return this->config[parameter];
}

const char* Config::getString(const char* parameter) {
    // ensure sure the value was set by the config file
    assert(this->configString.count(parameter) == 1);

    return this->configString[parameter].c_str();
}

// load config file in Slic3r format
void Config::loadConfig(const char* filename) {
    printf("Loading config %s...\n", filename);
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        printf("open file fail\n");
    }
    char line[256];
    while (!feof(file)) {
        if (fgets(line, sizeof(line), file)) {
            char name[256];
            float value;
            char valueString[1024];
            // scan for one 'name = value' entry
            // store single float values
            int found = sscanf(line, "%s = %e", name, &value);
            if (found)
                this->config[name] = value;
            // also store value as string, useful for strings and other non-float parameters
            found = sscanf(line, "%s = %[^\n]", name, valueString);
            if (found) {
                int pos;
                std::string s = valueString;
                // replace \n by real newlines
                std::string newlineEscape = "\\n";
                while ((pos = s.find(newlineEscape)) != -1) {
                    s.erase(pos, newlineEscape.length());
                    s.insert(pos, "\n");
                }
                this->configString[name] = s;
            }
        }
    }
    fclose(file);
}

