#include "log_sink.h"

static LogSinkFn s_sink = nullptr;

void log_sink_register(LogSinkFn fn)
{
    s_sink = fn;
}

void log_sink_send(const String &msg)
{
    if (s_sink)
        s_sink(msg);
}
