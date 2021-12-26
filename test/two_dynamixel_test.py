import numpy as np
from dynamixel_sdk import *


ADDR_AX_TORQUE_ENABLE = 24
ADDR_AX_GOAL_POSITION = 30
ADDR_AX_PRESENT_POSITION = 36

LEN_AX_GOAL_POSITION = 2
LEN_AX_PRESENT_POSITION = 2

PROTOCOL_VERSION = 1.0

DXL_ID = [3, 5]
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(
    port=portHandler,
    ph=packetHandler,
    start_address=ADDR_AX_GOAL_POSITION,
    data_length=LEN_AX_GOAL_POSITION
)

if portHandler.openPort():
    print("opened the port ", DEVICENAME)
else:
    print("port failed")
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Set the baudrate as ", BAUDRATE)
else:
    print("baudrate failed")
    quit()


for i in range(len(DXL_ID)):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        port=portHandler,
        dxl_id=DXL_ID[i],
        address=ADDR_AX_TORQUE_ENABLE,
        data=TORQUE_ENABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error:
        print(packetHandler.getRxPacketError(dxl_error))
    else:
        print("enabled DXL#%02d" % DXL_ID[i])


while True:
    # write goal position of joints
    if input("type not 'q' but anything to continue : ") == 'q':
        break

    dxl_goal_pos = [1]*len(DXL_ID)
    param_goal_pos = [[]]*len(DXL_ID)
    dxl_present_pos = [1]*len(DXL_ID)

    for i in range(len(DXL_ID)):
        dxl_goal_pos[i] = int(input("DXL#%02d's goal position : " % DXL_ID[i]))
        param_goal_pos[i] = [
            DXL_LOBYTE(dxl_goal_pos[i]),
            DXL_HIBYTE(dxl_goal_pos[i])
        ]

    for i in range(len(DXL_ID)):
        dxl_addparam_result = groupSyncWrite.addParam(
            dxl_id=DXL_ID[i],
            data=param_goal_pos[i]
        )
        if not dxl_addparam_result:
            print("DXL#%d groupSyncWrite addparam failed" % DXL_ID[i])
            quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))

    groupSyncWrite.clearParam()

    # read present position of joints
    while True:
        for i in range(len(DXL_ID)):
            dxl_present_pos[i], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
                port=portHandler,
                dxl_id=DXL_ID[i],
                address=ADDR_AX_PRESENT_POSITION
            )
            if dxl_comm_result != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error:
                print(packetHandler.getRxPacketError(dxl_error))

        for i in range(len(DXL_ID)):
            print("DXL#%02d GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], dxl_goal_pos[i], dxl_present_pos[i]), end='\t')
        print()

        if (np.abs(np.array(dxl_goal_pos) - np.array(dxl_present_pos)) < 3).all():
            break


for i in range(len(DXL_ID)):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        port=portHandler,
        dxl_id=DXL_ID[i],
        address=ADDR_AX_TORQUE_ENABLE,
        data=TORQUE_DISABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
    else:
        print("disabled DXL#%02d" % DXL_ID[i])


portHandler.closePort()
