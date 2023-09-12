#pragma once
#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <iostream>
#include <sstream>

#include <cstdarg> /* va_end(), va_list, va_start(), vprintf() */
#include <cstdio> /* vprintf() */

// Buffered debug output
// TODO: flush buffer after buffering X characters.
class DebugOut {
public:
    static DebugOut& Instance()
    {
        // Since it's a static variable, if the class has already been created,
        // It won't be created again.
        // And it **is** thread-safe in C++11.

        static DebugOut myInstance;

        // Return a reference to our instance.
        return myInstance;
    }

    // delete copy and move constructors and assign operators
    DebugOut(DebugOut const&) = delete;             // Copy construct
    DebugOut(DebugOut&&) = delete;                  // Move construct
    DebugOut& operator=(DebugOut const&) = delete;  // Copy assign
    DebugOut& operator=(DebugOut&&) = delete;      // Move assign

protected:
    DebugOut(std::ostream& out = std::cout) : m_Out(out) {}
    ~DebugOut() {
        m_Stream << "\n";
        m_Out << m_Stream.rdbuf();
        m_Out.flush();
    }
public:

    // cout style debug print function.
    template <class T>
    DebugOut& operator<<(const T& thing) { m_Stream << thing; return *this; }

    // printf style debug print function
    DebugOut& printf(const char* filename, int lineNo, const char* funcName, const char* fmt...) {
        m_Stream << filename << ":" << lineNo << " (" << funcName << ") ";
        char buffer[1024];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buffer, 1024, fmt, ap);
        va_end(ap);
        m_Stream << buffer;
        return *this;
    }
private:
    std::stringstream m_Stream;
    std::ostream& m_Out;
};


#ifndef DEBUG_ENABLED
#define DEBUG(_) do {} while(0)
#define DPRINTF(format, args) do {} while(0)
#define XTRACE
#else
#define DEBUG(Message_) DebugOut::Instance() << __FILE__ << ":" << __LINE__ << " (" << __PRETTY_FUNCTION__ << ") " << Message_
#define DPRINTF(format, ...) DebugOut::Instance().printf(__FILE__, __LINE__, __PRETTY_FUNCTION__, format, ##__VA_ARGS__)
#endif


#endif //__DEBUG_H__
