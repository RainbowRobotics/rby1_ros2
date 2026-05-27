# Control manager state constants.
int32 STATE_NONE = 0
int32 STATE_IDLE = 1
int32 STATE_ENABLE = 2
int32 STATE_EXECUTING = 3
int32 STATE_MAJOR_FAULT = 4
int32 STATE_MINOR_FAULT = 5

int32 control_manager_state

# Brake state grouped by robot component.
BrakeState brake_state

# Tool flange connection flags: [left, right].
bool[] tool_flange_state

# Emergency stop state. True means pressed or active.
bool emo_state

# Center of mass in meters.
float64[3] center_of_mass
