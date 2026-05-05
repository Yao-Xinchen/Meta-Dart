#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <unistd.h>

namespace mdlog {

enum class Level {
    Info,
    Ok,
    Warn,
    Error,
    State,
    Motion,
    Event,
};

inline bool use_color()
{
    static const bool enabled = (std::getenv("NO_COLOR") == nullptr) && isatty(STDOUT_FILENO);
    return enabled;
}

inline const char* color(Level level)
{
    if (!use_color())
        return "";
    switch (level) {
    case Level::Info:   return "\033[36m";
    case Level::Ok:     return "\033[32m";
    case Level::Warn:   return "\033[33m";
    case Level::Error:  return "\033[31m";
    case Level::State:  return "\033[35m";
    case Level::Motion: return "\033[34m";
    case Level::Event:  return "\033[1;37m";
    default:            return "";
    }
}

inline const char* label(Level level)
{
    switch (level) {
    case Level::Info:   return "SYSTEM";
    case Level::Ok:     return "SYSTEM";
    case Level::Warn:   return "WARN";
    case Level::Error:  return "ERROR";
    case Level::State:  return "SEARCHING";
    case Level::Motion: return "ASSEMBLING";
    case Level::Event:  return "SYSTEM";
    default:            return "SYSTEM";
    }
}

inline std::mutex& output_mutex()
{
    static std::mutex mutex;
    return mutex;
}

inline void write(Level level, const char* component, const char* fmt, ...)
{
    (void)component;
    std::lock_guard<std::mutex> lock(output_mutex());

    FILE* stream = (level == Level::Error || level == Level::Warn) ? stderr : stdout;
    const char* reset = use_color() ? "\033[0m" : "";

    std::fprintf(stream, "%s【%s】 ", color(level), label(level));

    va_list args;
    va_start(args, fmt);
    std::vfprintf(stream, fmt, args);
    va_end(args);

    std::fprintf(stream, "%s\n", reset);
    std::fflush(stream);
}

inline void write_custom(Level level, const char* tag, const char* fmt, ...)
{
    std::lock_guard<std::mutex> lock(output_mutex());

    FILE* stream = (level == Level::Error || level == Level::Warn) ? stderr : stdout;
    const char* reset = use_color() ? "\033[0m" : "";

    std::fprintf(stream, "%s【%s】 ", color(level), tag);

    va_list args;
    va_start(args, fmt);
    std::vfprintf(stream, fmt, args);
    va_end(args);

    std::fprintf(stream, "%s\n", reset);
    std::fflush(stream);
}

template <typename... Args>
inline void info(const char* component, const char* fmt, Args... args)
{
    write(Level::Info, component, fmt, args...);
}

template <typename... Args>
inline void ok(const char* component, const char* fmt, Args... args)
{
    write(Level::Ok, component, fmt, args...);
}

template <typename... Args>
inline void warn(const char* component, const char* fmt, Args... args)
{
    write(Level::Warn, component, fmt, args...);
}

template <typename... Args>
inline void error(const char* component, const char* fmt, Args... args)
{
    write(Level::Error, component, fmt, args...);
}

template <typename... Args>
inline void state(const char* component, const char* fmt, Args... args)
{
    write(Level::State, component, fmt, args...);
}

template <typename... Args>
inline void motion(const char* component, const char* fmt, Args... args)
{
    write(Level::Motion, component, fmt, args...);
}

template <typename... Args>
inline void event(const char* component, const char* fmt, Args... args)
{
    write(Level::Event, component, fmt, args...);
}

template <typename... Args>
inline void system(const char* fmt, Args... args)
{
    write_custom(Level::Info, "SYSTEM", fmt, args...);
}

template <typename... Args>
inline void searching(const char* fmt, Args... args)
{
    write_custom(Level::State, "SEARCHING", fmt, args...);
}

template <typename... Args>
inline void assembling(const char* fmt, Args... args)
{
    write_custom(Level::Motion, "ASSEMBLING", fmt, args...);
}

template <typename... Args>
inline void catch_(const char* fmt, Args... args)
{
    write_custom(Level::Event, "CATCH", fmt, args...);
}

template <typename... Args>
inline void disturb(const char* fmt, Args... args)
{
    write_custom(Level::Warn, "DISTURB", fmt, args...);
}

}  // namespace mdlog
