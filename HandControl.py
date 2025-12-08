import dynamixel_sdk as dxlSDK
import sys, math, time

global deviceSerial, B_Rate
deviceSerial = '/dev/ttyUSB0'
B_Rate = 57600

#Communication for sending commands through Dynamixel motor controllers
class DXL_Coms(object):
    def __init__ (self, device_name = deviceSerial, b_rate = B_Rate):
        BAUDRATE = b_rate
        DEVICENAME = device_name
        PROTOCOL_VERSION = 2.0
        #Opening Ports
        try:
            if self.port_handler.openPort():
                print("Succeeded to open the port")
                self.portHandler_Check_Pass = True
            else:
                print("Failed to open the port")
                print("System will run WITHOUT Real Mortor")
                self.portHandler_Check_Pass = False
                self.closeHandler()
        except Exception as e:   #error logs 
            print(e)
            print("Failed to open the port")
            print("System will run WITHOUT Real Mortor")
            self.portHandler_Check_Pass = False
            return
        # Set port baudrate
        if self.portHandler_Check_Pass:
            if self.port_handler.setBaudRate(BAUDRATE):
                print("Succeeded to change the baudrate")
                self.portHandler_Check_Pass = True
            else:
                print("Failed to change the baudrate")
                print("System will run WITHOUT Real Mortor")
                self.portHandler_Check_Pass = False
        
        self.motors = []
        self.parm = []

    def createMotor(self, name, motor_number = 1):
        if motor_number not in [motor.DXL_ID for motor in self.motors]:
            motor = DXL_Motor(self.port_handler, self.packet_handler, motor_number)
            motor.pingMotor()
            if motor.connected: 
                motor.name= name
                self.motors.append(motor)
                self.addAllBuckParameter()
                return motor
            else: 
                print("motor {0} connect error". format (motor_number))
                return None
        else:
            print ("Motor {0} already exist" .format(motor_number))
            for motor in self.motors: 
                if motor.DXL_ID == motor_number:
                    return motor
            
    def activateIndirectMode(self):
        for motors in self.motors:
            motor.activateIndirectMode()
        self.addAllBukK
        
    def sentAllCmd(self):
        for motor in self.motors:
            for msg in motor.msg_sent:
                _ = self.groupBulkWrite.addParam(*msg)
            motor.msg_sent = list()
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            print("DXL: sentAllCmd Error: {0}".format(self.packet_handler.getTxRxResult(dxl_comm_result)))
        # while result != dxlSDK.COMM_SUCCESS:
        #     result = self.groupBulkWrite.txPacket()
        #     print(self.packet_handler.getTxRxResult(result))
        self.groupBulkWrite.clearParam()

    def disableAllMotor(self):
        self.groupBulkWrite.clearParam()
        for motor in self.motors:
            self.groupBulkWrite.addParam(motor.DXL_ID, 64, 1, [0])
        result = self.groupBulkWrite.txPacket()
        if result == dxlSDK.COMM_SUCCESS:
            self.groupBulkWrite.clearParam()
        else:
            print(self.packet_handler.getTxRxResult(result))
            self.sentAllCmd()
    def closeHandler(self):
        self.port_handler.closePort()

    def sentCommand(self):
        self.groupBulkWrite.txPacket()
        self.groupBulkWrite.clearParam()

    def readHardwareError(self):
        for motor in self.motors:
            motor.readHardwareError()

    def rebootAllMotor(self):
        for motor in self.motors:
            motor.rebootMotor()
        time.sleep(3)
        self.__communicate_error_count = 0

    def checkErrorCount(self):
        return self.__communicate_error_count





class DXL_Motor(object):
    def __init___(self, port_h, package_h, Motor_num =1):
        #Control Table Default Values (set for XL330)
        self.name = None
        self.indirect_mode = False
        self.connected = False
        self.write_addr_info= {
            'GOAL_POSITION': {'ADDR': 116, 'LEN': 4},
            'GOAL_VELOCITY': {'ADDR': 104, 'LEN': 4}
        }
        self.read_addr_info = {
            'TORQUE_ENABLE':      {'ADDR':  64 ,'LEN': 1},
            'HARDWARE_ERR':       {'ADDR':  70 ,'LEN': 1},
            'PRESENT_CURRENT':    {'ADDR': 126 ,'LEN': 2},
            'PRESENT_VELOCITY':   {'ADDR': 128 ,'LEN': 4},
            'PRESENT_POSITION':   {'ADDR': 132 ,'LEN': 4},
            'PRESENT_TEMPERATURE': {'ADDR': 146 ,'LEN': 1}
        }
        self.indirect_read_addr_info = dict()
        self.packet_h = package_h
        self.port_h = port_h

        #Initial Motor Configuration
        self.DXL_ID                      = Motor_number      # Dynamixel ID : 1

        self.MortorStatus                = 0
        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 0                # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MAXIMUM_VELOCITY_VALUE  = 1023
        self.DXL_MINIMUM_VELOCITY_VALUE  = -1023
        self.DXL_MINIMUM_PWM_VALUE = -885
        self.DXL_MAXIMUM_PWM_VALUE = 885

        self.index = 0
        self.dxl_goal_position = [self.DXL_MINIMUM_POSITION_VALUE, self.DXL_MAXIMUM_POSITION_VALUE]
        self.all_data_len      = 0
        self.start_addr        = 0
        self.data              = list()
        self.acc_profile       = 0

        self.OPERATING_MODE             = None
        self.TORQUE_ENABLE_value        = 0
        self.PRESENT_CURRENT_value      = 0
        self.PRESENT_VELOCITY_value     = 0
        self.PRESENT_POSITION_value     = 0
        self.PRESENT_TEMPERTURE_value   = 0
        self.HARDWARE_ERR_value         = 0

        self.msg_sent = list()
        self.checkOperatingMode()

    def checkOperatingMode(self):
        mode_number, read_success = self.directReadData(*OPERATE_MODE_ADD_LEN)
        if read_success:
            self.OPERATING_MODE = mode_number

    def switchMode(self, mode = 'position'):
        if not self.torqueEnabled():
            if mode == 'position':
                mode = POSITION_MODE
            elif mode == 'velocity':
                mode = VELOCITY_MODE
            elif mode == 'pwm':
                mode = PWM_MODE
            switch_success = self.directWriteData(mode, *OPERATE_MODE_ADD_LEN)
            if switch_success:
                self.checkOperatingMode()
                if self.OPERATING_MODE == mode:
                    print ("Motor {0} OP Mode Change to {1} successfully" .format(self.DXL_ID, mode))
                    return True
                else: 
                    print("Motor {0} OP Mode Change Unsuccessfully" .format(self.DXL_ID))
                    return False
            else: 
                print("Mode Not Changed")
                if self.TORQUE_ENABLE_value == 1:
                    print("Disable Motor {0} first" .format(self.DXL_ID))
                    return False

    def torqueEnabled(self):
        torque_enable, read_success = self.directReadData(*TORQUE_ADDR_LEN)
        if read_success:
            self.TORQUE_ENABLE_value = torque_enable
            return True if torque_enable == 1 else False
        else:
            print("read fail")
            return None
        
    def enableMotor(self):
        tqe_on = self.directWriteData(TORQUE_ENABLE, *TORQUE_ADDR_LEN)
        # led_on = self.directWriteData(LED_OFF, *LED_ADDR_LEN)
        if tqe_on:
            print("Motor{0} is successfully armed".format(self.DXL_ID))
        elif not tqe_on:
            if self.torqueEnabled():
                print("Motor{0} armed with error".format(self.DXL_ID))
            else:
                print("Motor{0} not armed".format(self.DXL_ID))

    def activateIndirectMode(self):
        self.indirect_mode = True
        INDIRECT_START = 168
        INDIRECT_DATA_START = 224
        addr_prob = INDIRECT_START
        indirect_addr = INDIRECT_DATA_START
        for data_name, addr_info in self.read_addr_info.items():
            #Create Indirect ADDR information
            # future_to_read, dxl_comm_result, dxl_error = self.packet_h.read2ByteTxRx(
            #         self.port_h, self.DXL_ID, addr_prob
            #         )
            self.indirect_read_addr_info[data_name] = {
                'ADDR': indirect_addr, 'LEN': addr_info['LEN']
            }
            indirect_w_success = None
            for addr_shift in range(addr_info['LEN']):
                indirect_w_success = self.directWriteData(
                    addr_info['ADDR'] + addr_shift, addr_prob, 2
                )
                # dxl_comm_result, dxl_error = self.packet_h.write2ByteTxRx(
                #     self.port_h, self.DXL_ID, addr_prob, addr_info['ADDR'] + addr_shift
                #     )

                if indirect_w_success:
                    print("data [{0}] bit[{1}] of motor {2}, is set to {3} indirect address".format(
                        data_name, addr_shift+1, self.DXL_ID, indirect_addr
                    ))
                    addr_prob += 2
                    indirect_addr += 1
                else:
                    print("Indirect Address Faild in Motor{0}".format(self.DXL_ID))
                    self.indirect_mode = False
                    return
    def disableMotor(self):
        tqe_off = self.directWriteData(TORQUE_DISABLE, *TORQUE_ADDR_LEN)
        # led_off = self.directWriteData(LED_OFF, *LED_ADDR_LEN)
        if tqe_off:
            print("Motor{0} disarmed SUCCESSFULLY".format(self.DXL_ID))
        else:
            if not tqe_off:
                self.directWriteData(0, *TORQUE_ADDR_LEN, True)
                print("Motor{0} disarmed UNSUCCESSFULLY".format(self.DXL_ID))
            # if not led_off: self.directWriteData(0, *LED_ADDR_LEN, True)

    def writeVelocity(self, value):
        if self.OPERATING_MODE == VELOCITY_MODE:
            ADDR = 104
            LEN = 4
            data = [
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_HIWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_HIWORD(value))
            ]
            # self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
            if value >= self.DXL_MINIMUM_VELOCITY_VALUE and value <= self.DXL_MAXIMUM_VELOCITY_VALUE:
                self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
                # _ = DXL_Conmunication.groupBulkWrite.addParam(self.DXL_ID, ADDR, LEN, data)
            else:
                print("Commond exceed maximum range")
        else:
            print("Operating Mode Error while setting velocity")
    def writePosition(self, value):
        if self.OPERATING_MODE == POSITION_MODE:
            ADDR = 116
            LEN = 4
            data = [
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_HIWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_HIWORD(value))
            ]
            # self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
            if value >= self.DXL_MINIMUM_POSITION_VALUE and value <= self.DXL_MAXIMUM_POSITION_VALUE:
                self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
                # _ = DXL_Conmunication.groupBulkWrite.addParam(self.DXL_ID, ADDR, LEN, data)
            else:
                print("Commond exceed maximum range")
        else:
            print("Operating Mode Error while setting position")

    def writePWM(self, value):
        if self.OPERATING_MODE == PWM_MODE:
            ADDR = 100
            LEN = 2
            data = [
                dxlSDK.DXL_LOBYTE(dxlSDK.DXL_LOWORD(value)),
                dxlSDK.DXL_HIBYTE(dxlSDK.DXL_LOWORD(value)),
                # dxlSDK.DXL_LOBYTE(dxlSDK.DXL_HIWORD(value)),
                # dxlSDK.DXL_HIBYTE(dxlSDK.DXL_HIWORD(value))
            ]
            # self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
            if value >= self.DXL_MINIMUM_PWM_VALUE and value <= self.DXL_MAXIMUM_PWM_VALUE:
                self.msg_sent.append((self.DXL_ID, ADDR, LEN, data))
                # _ = DXL_Conmunication.groupBulkWrite.addParam(self.DXL_ID, ADDR, LEN, data)
            else:
                print("Commond exceed maximum range")
        else:
            print("Operating Mode Error while setting position")


    def MotorCorrection(self):
        pass

    def infoParam(self,name):
        ADDR, LEN = None, None
        if name == 'torque':
            ADDR, LEN = self.read_addr_info['TORQUE_ENABLE']['ADDR'], self.read_addr_info['TORQUE_ENABLE']['LEN']
        if name == 'current':
            ADDR, LEN = self.read_addr_info['PRESENT_CURRENT']['ADDR'], self.read_addr_info['PRESENT_CURRENT']['LEN']
        if name == 'velocity':
            ADDR, LEN = self.read_addr_info['PRESENT_VELOCITY']['ADDR'], self.read_addr_info['PRESENT_VELOCITY']['LEN']
        if name == 'position':
            ADDR, LEN = self.read_addr_info['PRESENT_POSITION']['ADDR'], self.read_addr_info['PRESENT_POSITION']['LEN']
        if name == 'temperture':
            ADDR, LEN = self.read_addr_info['PRESENT_TEMPERTURE']['ADDR'], self.read_addr_info['PRESENT_TEMPERTURE']['LEN']
        return self.DXL_ID, ADDR, LEN

    def addRequestValue(self, name, addr, dlen):
        self.read_addr_info[name] = {'ADDR': addr, 'LEN': dlen}
        setattr(self, name + "_value", 0)

    def updateValue(self):
        for name, info in self.indirect_read_addr_info.items() if self.indirect_mode else self.read_addr_info.items():
            shifted_address = info['ADDR'] - self.start_addr
            byte_data = self.data[shifted_address:shifted_address+info['LEN']]
            if info['LEN'] == 1:
                value = byte_data[0]
            elif info['LEN'] == 2:
                value = dxlSDK.DXL_MAKEWORD(byte_data[0],byte_data[1])
            elif info['LEN'] == 4:
                value = dxlSDK.DXL_MAKEDWORD(
                    dxlSDK.DXL_MAKEWORD(byte_data[0],byte_data[1]),
                    dxlSDK.DXL_MAKEWORD(byte_data[2],byte_data[3])
                )
            else:
                value = byte_data
            setattr(self, name + "_value", value)
            if self.PRESENT_CURRENT_value >= 32768:
                self.PRESENT_CURRENT_value = self.PRESENT_CURRENT_value-65535
            if self.PRESENT_VELOCITY_value >= (2**32)/2:
                self.PRESENT_VELOCITY_value = self.PRESENT_VELOCITY_value - (2**32-1)
            if self.PRESENT_POSITION_value >= (2**32)/2:
                self.PRESENT_POSITION_value = self.PRESENT_POSITION_value - (2**32-1)
        if self.HARDWARE_ERR_value == 8:
            self.PRESENT_CURRENT_value = None
            self.PRESENT_POSITION_value = None
            self.PRESENT_VELOCITY_value = None

    def directReadData(self, add, len, print_msg=True) -> tuple[int, bool]:
        value, com_err_msg, dxl_err_msg = None, None, None
        func_name = "read{0}ByteTxRx".format(len)
        func_ = getattr(self.packet_h,func_name)
        value, dxl_comm_result, dxl_error = func_(self.port_h, self.DXL_ID, add)
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            com_err_msg = self.packet_h.getTxRxResult(dxl_comm_result)
        elif dxl_error != 0:
            dxl_err_msg = self.packet_h.getRxPacketError(dxl_error)
        else:
            return value, True
        if com_err_msg is not None or dxl_err_msg is not None:
            if com_err_msg and print_msg: print("DXL: directReadData Error: {0} at ID: {1}".format(com_err_msg, self.DXL_ID))
            if dxl_err_msg and print_msg: print("DXL: directReadData Error: {0} at ID: {1}".format(dxl_err_msg, self.DXL_ID))
            return value, False

    def directWriteData(self, data, add, len, print_msg=True):
        com_err_msg, dxl_err_msg = None, None
        func_name = "write{0}ByteTxRx".format(len)
        func_ = getattr(self.packet_h, func_name)
        dxl_comm_result, dxl_error = func_(self.port_h, self.DXL_ID, add, data)
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            com_err_msg = self.packet_h.getTxRxResult(dxl_comm_result)
        elif dxl_error != 0:
            dxl_err_msg = self.packet_h.getRxPacketError(dxl_error)
        else:
            return True
        if com_err_msg is not None or dxl_err_msg is not None:
            if com_err_msg and print_msg: print("DXL: directWriteData Error: {0} at ID: {1}".format(com_err_msg, self.DXL_ID))
            if dxl_err_msg and print_msg: print("DXL: directWriteData Error: {0} at ID: {1}".format(dxl_err_msg, self.DXL_ID))
            return False
            
    def setVelocity(self, v_cmd):
        if self.OPERATING_MODE == VELOCITY_MODE:
            addr_len = self.write_addr_info['GOAL_VELOCITY']
            if v_cmd <= self.DXL_MAXIMUM_VELOCITY_VALUE and v_cmd >= self.DXL_MINIMUM_VELOCITY_VALUE:
                self.directWriteData(v_cmd, addr_len['ADDR'], addr_len['LEN'],True)
            else:
                print("Command out off range")
        else:
            print("Mode Error while write velocity in {0} mode".format(self.OPERATING_MODE))
    
    def setPosition(self, p_cmd):
        if self.OPERATING_MODE == POSITION_MODE:
            addr_len = self.write_addr_info['GOAL_POSITION']
            if p_cmd <= self.DXL_MAXIMUM_POSITION_VALUE and p_cmd >= self.DXL_MINIMUM_POSITION_VALUE:
                self.directWriteData(p_cmd, addr_len['ADDR'], addr_len['LEN'], True)
            else:
                print("Command out off range")
        else:
            print("Mode Error while write position in {0} mode".format(self.OPERATING_MODE))

    def setAccelerationProfile(self, profile: int)->None:
        self.acc_profile = profile
        ADDR = 108
        LEN  = 4
        self.directWriteData(profile, add=ADDR, len=LEN)

    def pingMotor(self):
        dxl_model_number, dxl_comm_result, dxl_error = self.packet_h.ping(self.port_h, self.DXL_ID)
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            print("DXL: Ping Error: {0} at ID:{1}".format(self.packet_h.getTxRxResult(dxl_comm_result), self.DXL_ID))
        elif dxl_error != 0:
            print("DXL: Ping Error: {0} at ID:{1}".format(self.packet_h.getRxPacketError(dxl_error), self.DXL_ID))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.DXL_ID, dxl_model_number))
            self.connected = True

    def rebootMotor(self):
        dxl_comm_result, dxl_error = self.packet_h.reboot(self.port_h, self.DXL_ID)
        if dxl_comm_result != dxlSDK.COMM_SUCCESS:
            print("ID:{0} reboot Error: {1}".format(self.DXL_ID,self.packet_h.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            print("ID:{0} reboot Error: {1}".format(self.DXL_ID,self.packet_h.getRxPacketError(dxl_error)))

        print("[ID:{0}] reboot Succeeded".format(self.DXL_ID))

   
class MyGroupBucketRead(dxlSDK.GroupBulkRead):

    def __init__(self,port_handler, packet_handler):
        super(MyGroupBucketRead, self).__init__(port_handler,packet_handler)

    def getData(self, dxl_id, address, data_length):
        PARAM_NUM_DATA = 0
        PARAM_NUM_ADDRESS = 1
        if not self.isAvailable(dxl_id, address, data_length):
            return 0

        start_addr = self.data_dict[dxl_id][PARAM_NUM_ADDRESS]

        if data_length == 1:
            return self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr]
        elif data_length == 2:
            return dxlSDK.DXL_MAKEWORD(
                self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr],
                self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1]
            )
        elif data_length == 4:
            return dxlSDK.DXL_MAKEDWORD(
                dxlSDK.DXL_MAKEWORD(
                    self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 0],
                    self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1]),
                dxlSDK.DXL_MAKEWORD(
                    self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 2],
                    self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 3])
                )
        else:
            return self.data_dict[dxl_id][PARAM_NUM_DATA]
            
# Hand-Related Controls

#Hand Motor Initialisation
dynamixel = DXL_Coms(deviceSerial,B_Rate)
ring = dynamixel.createMotor("ring",motor_number = 1)
thumb = dynamixel.createMotor("thumb",motor_number = 2)
middle = dynamixel.createMotor("middle",motor_number = 3)
pinky = dynamixel.createMotor("pinky",motor_number = 4)
pointer= dynamixel.createMotor("pointer",motor_number = 5)

motor_list = [ring, thumb, middle, pinky, pointer]
for motor in motor_list:
    motor.switchMode('position') #default control mode set to position
    motor.enableMotor()
    
"""
    Hardware Limits on Motors Due to mechanical constraints of the hand, the motors cannot reach their full range of motion.
    This function opens the hand by moving the motors to their maximum position values.
    
    Values for Maximum Pos of Each Finger (Open Hand)
    Thumb: 0
    Pointer: 2074
    Middle:1972
    Ring: 1974
    Pinky: 2475
    
    Values for Minimum Pos of Each Finger (Closed Fist)
    Thumb: 410
    Pointer: 2590
    Middle: 1537
    Ring: 2551
    Pinky: 1920
"""    
def handOpen():    
    ring.writePosition(1974)
    thumb.writePosition(0)
    middle.writePosition(1972)
    pinky.writePosition(2475)
    pointer.writePosition(2074)
    dynamixel.sendAllCmd()
    dynamixel.updateMotorData()
    
def handClose():
    ring.writePosition(2550)
    thumb.writePosition(360)
    middle.writePosition(1600)
    pinky.writePosition(2400)
    pointer.writePosition(2500)
    dynamixel.sendAllCmd()
    dynamixel.updateMotorData()
    
def MotorPosControl(motorName, movement):
    motorName.switchMode('position')
    motorName.writePosition(movement)
    dynamixel.sendAllCmd()
    dynamixel.updateMotorData()

def checkAllPos():
    for motor in motor_list:
        print(motor.DXL_ID)
        print(motor.infoParam('position'))
        
        

def main(): 
    print("This is DXL_Coms module test file.")
    print("Welcome, Main Start")
    handOpen()
    checkAllPos()
    input("Hand set to Open by Default, Please Press Enter to Close Hand")
    handClose()
    checkAllPos()
    for motor in motor_list:
        motor.disableMotor()
    print("Program Exit")



if __name__ == "__main__":
    main()
