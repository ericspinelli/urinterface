# CONSTANTS
# Socket data handling constants
BUFFER_SIZE = 4096
LENGTH_SIZE = 4
TYPE_SIZE = 1
HEADER_SIZE = LENGTH_SIZE + TYPE_SIZE

# UR message types
MESSAGE_TYPE_VERSION = 20
MESSAGE_TYPE_ROBOT_STATE = 16

# Data type/size.  Set as constants for readability in DataObject sub-class implementation (datamap)
DOUBLE = ('!d', 8)
UINT64 = ('!Q', 8)
FLOAT = ('!f', 4)
UINT32 = ('!I', 4)
UINT = ('!I', 4)
INT = ('!i', 4)
UINT8 = ('!B', 1)
UCHAR = ('!B', 1)
CHAR = ('!b', 1)
BOOL = ('!?', 1)

# Subpackage type for robot state datastream
ROBOT_MODE = 0
JOINT_DATA = 1
TOOL_DATA = 2
MASTERBOARD_DATA = 3
CARTESIAN_DATA = 4
KINEMATICS_INFO = 5
CONFIG_DATA = 6
FORCE_MODE_DATA = 7
ADDITIONAL_INFO = 8
CALIBRATION_DATA = 9
SAFETY_DATA = 10
TOOL_COMM_DATA = 11
TOOL_MODE_DATA = 12
SINGULARITY_INFO = 13

# Names of each data type provided by the Secondary Interface datastream (only used for test purposes)
dataNameDict = {
    0: "Robot Mode Data",
    1: "Joint Data",
    2: "Tool Data",
    3: "Masterboard Data",
    4: "Cartesian Info",
    5: "Kinematics Info",
    6: "Configuration Data",
    7: "Force Mode Data",
    8: "Additional Info",
    9: "Calibration Data",
    10: "Safety Data",
    11: "Tool Comm Info",
    12: "Tool Mode Info",
    13: "Singularity Info"
}