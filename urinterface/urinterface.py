"""
Client software to connect to UR Controller Client Interface

URinterface class connects to UR controller's Secondary Client Interface (Port 30002).
URinterface objects can send URScript and receive the robot state datastream.
"""

# encoding: utf-8

__version__ = '0.1'
__author__ = 'Eric Spinelli'

import socket
import struct
from constants import *

class URinterface():
    """
    URinterface objects can connect to UR Client Interface
    Main methods are connect(), send(), and receive()
    """
    def __init__(self, host: str=None, port: int=None):
        """
        :param host: IP address of UR robot
        :param port: Port # of Client Interface (30001, 30002, 30003)
        """
        self._host = host
        self._port = port
        self.__socket = None
        self._dataReady = False

    def host(self, host: str=None):
        """
        :param host: If argument is passed, disconnect and set IP address
        :return host (str)
        """
        if host:
            self.disconnect()
            self._host = host
        return self._host

    def port(self, port: int=None):
        """
        :param port: if argument is passed, disconnect and set port #
        :return: port (int)
        """
        if port:
            self.disconnect()
            self._port = port
        return self._port

    def connect(self):
        """Open a socket connection with UR robot controller"""
        if not self._host or not self._host:
            raise URinterfaceError("Host and port not properly set")
        if self.isOpen():
            return
        try:
            self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__socket.connect((self._host, self._port))
        except (socket.timeout, socket.error):
            self.__socket = None
            raise

    def disconnect(self):
        """Close connection to UR controller"""
        if not self.isOpen():
            return
        self.__socket.close()
        self.__socket = None

    def isOpen(self):
        """
        Check if connection is established with UR controller
        :return: bool
        """
        return self.__socket

    def isReady(self):
        """
        Check if message type matching MESSAGE_TYPE_ROBOT_STATE was received by most receive recv() call
        Allows for ignoring other message types, such as received upon connection
        :return: bool
        """
        return self._dataReady

    def send(self, cmd):
        """
        Sends string to UR controller
        Appends newline (\n) control character if necessary
        :param cmd: string of URScript commands
        """
        if not self.isOpen():
            return
        if cmd[-1] != '\n':
            cmd.join('\n')
        self.__socket.send(cmd.encode())

    def recv(self):
        """
        Receives and does preliminary parsing of datastream from robot Client Interface
        :return: None; do not receive if not connected
        """
        if not self.__socket:
            return
        data = self.__socket.recv(BUFFER_SIZE)
        if not data:
            self.disconnect()
        else:
            self._dataReady = False
            data_pointer = 0
            overall_length = (struct.unpack('!i', data[data_pointer:data_pointer + LENGTH_SIZE]))[0]
            data_pointer += LENGTH_SIZE
            robot_messageType = data[4]
            data_pointer = HEADER_SIZE
            if int(robot_messageType) == MESSAGE_TYPE_VERSION:
                pass  # not implemented
            elif int(robot_messageType) == MESSAGE_TYPE_ROBOT_STATE:
                while data_pointer < overall_length:
                    subpkg_length = (struct.unpack('!i', data[data_pointer:data_pointer + LENGTH_SIZE]))[0]
                    subpkg_type = (struct.unpack('!B', data[data_pointer + LENGTH_SIZE:data_pointer + HEADER_SIZE]))[0]
                    content = data[data_pointer + HEADER_SIZE:data_pointer + subpkg_length]
                    if subpkg_type == ROBOT_MODE:
                        self.mode = RobotModeData.unpack(content)
                    elif subpkg_type == JOINT_DATA:
                        self.joint = JointData.unpack(content)
                    elif subpkg_type == TOOL_DATA:
                        self.tool = ToolData.unpack(content)
                    elif subpkg_type == MASTERBOARD_DATA:
                        self.masterboard = MasterboardData.unpack(content)
                    elif subpkg_type == CARTESIAN_DATA:
                        self.cartesian = CartesianData.unpack(content)
                    elif subpkg_type == KINEMATICS_INFO:
                        self.kinematics = KinematicData.unpack(content)
                    elif subpkg_type == CONFIG_DATA:
                        self.config = ConfigData.unpack(content)
                    elif subpkg_type == FORCE_MODE_DATA:
                        self.force = ForceData.unpack(content)
                    elif subpkg_type == ADDITIONAL_INFO:
                        self.additional = AdditionalData.unpack(content)
                    elif subpkg_type == CALIBRATION_DATA:
                        self.calibration = CalibrationData.unpack(content)
                    elif subpkg_type == SAFETY_DATA:
                        self.safety = SafetyData.unpack(content)
                    elif subpkg_type == TOOL_COMM_DATA:
                        self.toolcomm = ToolCommData.unpack(content)
                    elif subpkg_type == TOOL_MODE_DATA:
                        self.toolmode = ToolModeData.unpack(content)
                    elif subpkg_type == SINGULARITY_INFO:
                        self.singularity = SingularityData.unpack(content)

                    data_pointer += subpkg_length
                self._dataReady = True


class URinterfaceError(Exception):
    pass


class DataObject(object):
    """
    Parent class for all actual data types received from robot
    DataObjects created from raw data should use the classmethod unpack() rather than the constructor
    DataObjects created from already parsed data (type: list) should use constructor directly (generally only used for overloaded constructors e.g. JointData)
    Each sub-class contains datamap (type: tuple) which contains info on variable name and variable type/size
    Tuple is used over dictionary to preserve order.  Constants for type/size are used for readability
    order and type/size can be found in the Excel sheet here: https://www.universal-robots.com/articles/ur-articles/remote-control-via-tcpip/
    """
    def __init__(self, data):
        if data:
            self.__raw_data = data
            self.assign(data)

    @classmethod
    def unpack(cls, content):
        data_pointer = 0
        output = []
        for var in cls.datamap:
            data_type = var[1][0]
            data_length = var[1][1]
            output.append(struct.unpack(data_type, content[data_pointer:data_pointer + data_length])[0])
            data_pointer += data_length
        return cls(output)

    def assign(self, data):
        try:
            if len(data) == len(self.datamap):
                i = 0
                for var in self.datamap:
                    setattr(self, var[0], data[i])
                    i += 1
        except TypeError:
            print("DataObject requires data of type 'list' to construct")


# NOT IMPLEMENTED
# Data type chararray is NOT implemented
# class VersionMessage(DataObject):
#     datamap = (
#         ('timestamp', UINT64),
#         ('source', CHAR),
#         ('robotMessageType', CHAR),
#         ('projectNameSize', CHAR),
#         ('projectName', CHARARRAY),
#         ('majorVersion', UCHAR),
#         ('minorVersion', UCHAR),
#         ('bugfixVersion', INT),
#         ('buildNumber', INT),
#         ('buildDate', CHARARRAY)
#     )

class RobotModeData(DataObject):
    datamap = (
        ('timestamp', UINT64),
        ('isRealRobotConnected', BOOL),
        ('isRealRobotEnabled', BOOL),
        ('isRobotPowerOn', BOOL),
        ('isEmergencyStopped', BOOL),
        ('isProtectiveStopped', BOOL),
        ('isProgramRunning', BOOL),
        ('isProgramPaused', BOOL),
        ('robotMode', UCHAR),
        ('controlMode', UCHAR),
        ('targetSpeedFraction', DOUBLE),
        ('speedScaling', DOUBLE),
        ('targetSpeedFractionLimit', DOUBLE),
        ('internal', UCHAR)
    )


class JointData(DataObject):
    """
    JointData member variables are 'private' and should be accessed via SingleJoint member variables
    """
    datamap = (
        ('_j0_q_actual', DOUBLE), ('_j0_q_target', DOUBLE), ('_j0_qd_actual', DOUBLE),
        ('_j0_I_actual', FLOAT), ('_j0_V_actual', FLOAT), ('_j0_T_motor', FLOAT), ('_j0_T_micro', FLOAT), ('_j0_jointMode', UINT8),
        ('_j1_q_actual', DOUBLE), ('_j1_q_target', DOUBLE), ('_j1_qd_actual', DOUBLE),
        ('_j1_I_actual', FLOAT), ('_j1_V_actual', FLOAT), ('_j1_T_motor', FLOAT), ('_j1_T_micro', FLOAT), ('_j1_jointMode', UINT8),
        ('_j2_q_actual', DOUBLE), ('_j2_q_target', DOUBLE), ('_j2_qd_actual', DOUBLE),
        ('_j2_I_actual', FLOAT), ('_j2_V_actual', FLOAT), ('_j2_T_motor', FLOAT), ('_j2_T_micro', FLOAT), ('_j2_jointMode', UINT8),
        ('_j3_q_actual', DOUBLE), ('_j3_q_target', DOUBLE), ('_j3_qd_actual', DOUBLE),
        ('_j3_I_actual', FLOAT), ('_j3_V_actual', FLOAT), ('_j3_T_motor', FLOAT), ('_j3_T_micro', FLOAT), ('_j3_jointMode', UINT8),
        ('_j4_q_actual', DOUBLE), ('_j4_q_target', DOUBLE), ('_j4_qd_actual', DOUBLE),
        ('_j4_I_actual', FLOAT), ('_j4_V_actual', FLOAT), ('_j4_T_motor', FLOAT), ('_j4_T_micro', FLOAT), ('_j4_jointMode', UINT8),
        ('_j5_q_actual', DOUBLE), ('_j5_q_target', DOUBLE), ('_j5_qd_actual', DOUBLE),
        ('_j5_I_actual', FLOAT), ('_j5_V_actual', FLOAT), ('_j5_T_motor', FLOAT), ('_j5_T_micro', FLOAT), ('_j5_jointMode', UINT8),
    )

    def __init__(self, data):
        """
        Overload constructor for DataObject so that data for each joint is organized in a SingleJoint object
        """
        DataObject.__init__(self, data)
        self.j0 = SingleJoint([self._j0_q_actual, self._j0_q_target, self._j0_qd_actual, self._j0_I_actual, self._j0_V_actual,
                               self._j0_T_motor, self._j0_T_micro, self._j0_jointMode])
        self.j1 = SingleJoint([self._j1_q_actual, self._j1_q_target, self._j1_qd_actual, self._j1_I_actual, self._j1_V_actual,
                               self._j1_T_motor, self._j1_T_micro, self._j1_jointMode])
        self.j2 = SingleJoint([self._j2_q_actual, self._j2_q_target, self._j2_qd_actual, self._j2_I_actual, self._j2_V_actual,
                               self._j2_T_motor, self._j2_T_micro, self._j2_jointMode])
        self.j3 = SingleJoint([self._j3_q_actual, self._j3_q_target, self._j3_qd_actual, self._j3_I_actual, self._j3_V_actual,
                               self._j3_T_motor, self._j3_T_micro, self._j3_jointMode])
        self.j4 = SingleJoint([self._j4_q_actual, self._j4_q_target, self._j4_qd_actual, self._j4_I_actual, self._j4_V_actual,
                               self._j4_T_motor, self._j4_T_micro, self._j4_jointMode])
        self.j5 = SingleJoint([self._j5_q_actual, self._j5_q_target, self._j5_qd_actual, self._j5_I_actual, self._j5_V_actual,
                               self._j5_T_motor, self._j5_T_micro, self._j5_jointMode])


class SingleJoint(DataObject):
    """
    Restructures JointData objects
    """
    # Data type must be tuple but because SingleJoint is only called by constructor which does not require data format like @classmethod unpack()
    datamap = (('q_actual',), ('q_target',), ('qd_actual',), ('I_actual',), ('V_actual',), ('T_motor',), ('T_micro',), ('jointMode',))


class ToolData(DataObject):
    datamap = (
        ('analogInputRange0', UCHAR),
        ('analogInputRange1', UCHAR),
        ('analogInput0', DOUBLE),
        ('analogInput1', DOUBLE),
        ('toolVoltage48V', FLOAT),
        ('toolOutputVoltage', UCHAR),
        ('toolCurrent', FLOAT),
        ('toolTemperature', FLOAT),
        ('toolMode', UINT8)
    )


class MasterboardData(DataObject):
    """
    If EuroMap is installed, variables should be un-commented
    """
    datamap = (
        ('digitalInputBits', INT),
        ('digitalOutputBits', INT),
        ('analogInputRange0', UCHAR),
        ('analogInputRange1', UCHAR),
        ('analogInput0', DOUBLE),
        ('analogInput1', DOUBLE),
        ('analogOutputDomain0', CHAR),
        ('analogOutputDomain1', CHAR),
        ('analogOutput0', DOUBLE),
        ('analogOutput1', DOUBLE),
        ('masterBoardTemperature', FLOAT),
        ('robotVoltage48V', FLOAT),
        ('robotCurrent', FLOAT),
        ('masterIOCurrent', FLOAT),
        ('safetyMode', UCHAR),
        ('InReducedMode', UINT8),
        ('euromap67InterfaceInstalled', CHAR),
        # ('euromapInputBits', UINT32),
        # ('euromapOutputBits', UINT32),
        # ('euromapVoltage24V', FLOAT),
        # ('euromapCurrent', FLOAT),
        ('reserved1', UINT32),
        ('operationalModeSelectorInput', UINT8),
        ('threePositionEnablingDeviceInput', UINT8),
        ('reserved2', UCHAR)
    )


class CartesianData(DataObject):
    datamap = (
        ('x', DOUBLE),
        ('y', DOUBLE),
        ('z', DOUBLE),
        ('rx', DOUBLE),
        ('ry', DOUBLE),
        ('rz', DOUBLE),
        ('TCPOffsetX', DOUBLE),
        ('TCPOffsetY', DOUBLE),
        ('TCPOffsetZ', DOUBLE),
        ('TCPOffsetRx', DOUBLE),
        ('TCPOffsetRy', DOUBLE),
        ('TCPOffsetRz', DOUBLE)
    )


class KinematicData(DataObject):
    datamap = (
        ('checksum', UINT32),
        ('DHtheta', DOUBLE),
        ('DHa', DOUBLE),
        ('Dhd', DOUBLE),
        ('Dhalpha', DOUBLE),
        ('calibrationStatus', UINT32)
    )


class ConfigData(DataObject):
    datamap = (
        ('j0_jointMinLimit', DOUBLE), ('j0_jointMaxLimit', DOUBLE),
        ('j1_jointMinLimit', DOUBLE), ('j1_jointMaxLimit', DOUBLE),
        ('j2_jointMinLimit', DOUBLE), ('j2_jointMaxLimit', DOUBLE),
        ('j3_jointMinLimit', DOUBLE), ('j3_jointMaxLimit', DOUBLE),
        ('j4_jointMinLimit', DOUBLE), ('j4_jointMaxLimit', DOUBLE),
        ('j5_jointMinLimit', DOUBLE), ('j5_jointMaxLimit', DOUBLE),
        ('j0_jointMaxSpeed', DOUBLE), ('j0_jointMaxAcceleration', DOUBLE),
        ('j1_jointMaxSpeed', DOUBLE), ('j1_jointMaxAcceleration', DOUBLE),
        ('j2_jointMaxSpeed', DOUBLE), ('j2_jointMaxAcceleration', DOUBLE),
        ('j3_jointMaxSpeed', DOUBLE), ('j3_jointMaxAcceleration', DOUBLE),
        ('j4_jointMaxSpeed', DOUBLE), ('j4_jointMaxAcceleration', DOUBLE),
        ('j5_jointMaxSpeed', DOUBLE), ('j5_jointMaxAcceleration', DOUBLE),
        ('vJointDefault', DOUBLE),
        ('aJointDefault', DOUBLE),
        ('vToolDefault', DOUBLE),
        ('aToolDefault', DOUBLE),
        ('eqRadius', DOUBLE),
        ('j0_DHa', DOUBLE), ('j1_DHa', DOUBLE), ('j2_DHa', DOUBLE), ('j3_DHa', DOUBLE), ('j4_DHa', DOUBLE), ('j5_DHa', DOUBLE),
        ('j0_Dhd', DOUBLE), ('j1_Dhd', DOUBLE), ('j2_Dhd', DOUBLE), ('j3_Dhd', DOUBLE), ('j4_Dhd', DOUBLE), ('j5_Dhd', DOUBLE),
        ('j0_DHalpha', DOUBLE), ('j1_DHalpha', DOUBLE), ('j2_DHalpha', DOUBLE), ('j3_DHalpha', DOUBLE), ('j4_DHalpha', DOUBLE), ('j5_DHalpha', DOUBLE),
        ('j0_DHtheta', DOUBLE), ('j1_DHtheta', DOUBLE), ('j2_DHtheta', DOUBLE), ('j3_DHtheta', DOUBLE), ('j4_DHtheta', DOUBLE), ('j5_DHtheta', DOUBLE),
        ('masterboardVersion', UINT32),
        ('controllerBoxType', UINT32),
        ('robotType', UINT32),
        ('robotSubType', UINT32)
    )


class ForceData(DataObject):
    datamap = (
        ('Fx', DOUBLE),
        ('Fy', DOUBLE),
        ('Fz', DOUBLE),
        ('Frx', DOUBLE),
        ('Fry', DOUBLE),
        ('Frz', DOUBLE),
        ('robotdexterity', DOUBLE)
    )


class AdditionalData(DataObject):
    datamap = (
        ('tpButtonState', UCHAR),
        ('freedriveButtonEnabled', UCHAR),
        ('IOEnabledFreedrive', BOOL),
        ('reserved', UCHAR)
    )


class CalibrationData(DataObject):
    datamap = (
        ('Fx', DOUBLE),
        ('Fy', DOUBLE),
        ('Fz', DOUBLE),
        ('Frx', DOUBLE),
        ('Fry', DOUBLE),
        ('Frz', DOUBLE)
    )


class SafetyData(DataObject):
    """
    Datamap not published by UR
    """
    datamap = ()


class ToolCommData(DataObject):
    datamap = (
        ('toolCommunicationIsEnabled', BOOL),
        ('baudRate', INT),
        ('parity', INT),
        ('stopBits', INT),
        ('RxIdleChars', FLOAT),
        ('TxIdleChars', FLOAT)
    )


class ToolModeData(DataObject):
    datamap = (
        ('outputMode', UINT8),
        ('digtalOutputMode0', UINT8),
        ('digtalOutputMode1', UINT8)
    )


class SingularityData(DataObject):
    """
    Not received from URSim v5.8.0 and therefore untested
    """
    datamap = (
        ('scale', DOUBLE)
    )


def test():
    ursec = URinterface(host="192.168.254.130", port=30002)
    ursec.connect()
    for _ in range(10):
        ursec.recv()
        if ursec.isReady():
            for var in ursec.mode.datamap:
                print("{}: {}".format(var[0], getattr(ursec.mode, var[0])))
            print("time: {}, X: {}, Y: {}, Z: {}".format(ursec.mode.timestamp, ursec.cartesian.x, ursec.cartesian.y, ursec.cartesian.z))
            print("")
    print("")

    if ursec.isReady():
        # Mode 0: RobotMode
        print("RobotMode")
        for var in ursec.mode.datamap:
            print("{}: {}".format(var[0], getattr(ursec.mode, var[0])))
        print("")

        # Mode 1: Joints
        print("Joints")
        for var in ursec.joint.j0.datamap:
            print("j0: {}: {}".format(var[0], getattr(ursec.joint.j0, var[0])))
        for var in ursec.joint.j1.datamap:
            print("j1: {}: {}".format(var[0], getattr(ursec.joint.j1, var[0])))
        for var in ursec.joint.j2.datamap:
            print("j2: {}: {}".format(var[0], getattr(ursec.joint.j2, var[0])))
        for var in ursec.joint.j3.datamap:
            print("j3: {}: {}".format(var[0], getattr(ursec.joint.j3, var[0])))
        for var in ursec.joint.j4.datamap:
            print("j4: {}: {}".format(var[0], getattr(ursec.joint.j4, var[0])))
        for var in ursec.joint.j5.datamap:
            print("j5: {}: {}".format(var[0], getattr(ursec.joint.j5, var[0])))
        print("")

        # Mode 2: Tool
        print("Tool")
        for var in ursec.tool.datamap:
            print("{}: {}".format(var[0], getattr(ursec.tool, var[0])))
        print("")

        # Mode 3: Masterboard
        print("Masterboard")
        for var in ursec.masterboard.datamap:
            print("{}: {}".format(var[0], getattr(ursec.masterboard, var[0])))
        print("")

        # Mode 4: Cartesian
        print("Cartesian")
        for var in ursec.cartesian.datamap:
            print("{}: {}".format(var[0], getattr(ursec.cartesian, var[0])))
        print("")

        # Mode 5: Kinematics
        print("Kinematics")
        for var in ursec.kinematics.datamap:
            print("{}: {}".format(var[0], getattr(ursec.kinematics, var[0])))
        print("")

        # Mode 6: Config
        print("Config")
        for var in ursec.config.datamap:
            print("{}: {}".format(var[0], getattr(ursec.config, var[0])))
        print("")

        # Mode 7: Force
        print("Force")
        for var in ursec.force.datamap:
            print("{}: {}".format(var[0], getattr(ursec.force, var[0])))
        print("")

        # Mode 8: Additional
        print("Additional")
        for var in ursec.additional.datamap:
            print("{}: {}".format(var[0], getattr(ursec.additional, var[0])))
        print("")

        # Mode 9: Calibration
        print("Calibration")
        for var in ursec.calibration.datamap:
            print("{}: {}".format(var[0], getattr(ursec.calibration, var[0])))
        print("")

        # Mode 10: Safety
        print("Safety")
        for var in ursec.safety.datamap:
            print("{}: {}".format(var[0], getattr(ursec.safety, var[0])))
        print("")

        # Mode 11: Tool Comms
        print("Tool Comms")
        for var in ursec.toolcomm.datamap:
            print("{}: {}".format(var[0], getattr(ursec.toolcomm, var[0])))
        print("")

        # Mode 12: Tool Mode
        print("Tool Mode")
        for var in ursec.toolmode.datamap:
            print("{}: {}".format(var[0], getattr(ursec.toolmode, var[0])))
        print("")

        # Mode 13: Singularity
        # print("Singularity")
        # for var in ursec.singularity.datamap:
        #     print("{}: {}".format(var[0], getattr(ursec.singularity, var[0])))

        # Send URScript test
        # WARNING: ROBOT WILL MOVE
        # WARNING: Robot must be in Remote Control Mode
        # print("sending movej(p[0.1, 0.2, 0.1, 0, 3.14, 0])")
        # ursec.send("movej(p[0.1, 0.2, 0.1, 0, 3.14, 0])\n")

    ursec.disconnect()


if __name__ == '__main__':
    test()