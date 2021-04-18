#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "diagnostic_msgs/msg/diagnostic_array.h"

diagnostic_msgs__msg__DiagnosticArray *diagnositics_init(void);
void diagnostics_populate();

#endif // DIAGNOSTICS_H
