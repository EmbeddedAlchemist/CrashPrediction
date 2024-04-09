#pragma once

#include <exception>
#include <string>

class Exception {
public:
    const std::string message;
    inline Exception(const std::string &message) 
        :message(message) {};
    inline Exception() :Exception("") {};
};