MOTORS_PREFIX = 0x10  # Set all motors
GRIPPER_PREFIX = 0x94
MANIPULATOR_PREFIX = 0x84
FIRMWARE_VER_PREFIX = 0x99

CLUPI_PREFIX = 0x41
CLUPI_ADDR = 0x42

POSTFIX = [0x0D, 0x0A]

def firmware_ver():
    command = bytearray()
    command.append(FIRMWARE_VER_PREFIX)
    command.append(0x00)
    command.append(0x00)
    command.append(0x00)
    command.append(0x00)
    command.append(0x0D)
    command.append(0x0A)
    return command

def motors(payload):
    command = bytearray()
    command.append(MOTORS_PREFIX)
    command.extend(payload)
    command.extend(POSTFIX)
    return command

#	Read battery voltage (actualy, not the voltage but ADC reading)

def battery():
    command = [0x30]
    command.append(0x00)
    command.append(0x00)
    command.append(0x00)
    command.append(0x00)
    command.append(0x0D)
    command.append(0x0A)
    return command

#	Set servo values
def gripper(payload):
    command = bytearray()
    command.append(GRIPPER_PREFIX)
    command.extend(payload)
    command.append(0x00)
    command.append(0x00)
    command.extend(POSTFIX)
    return command


#	Set servo values
def manipulator(payload):
    command = bytearray()
    command.append(MANIPULATOR_PREFIX)
    command.extend(payload)
    command.append(0x0D)
    command.append(0x0A)
    return command

#   This frame is made specially for CLUPI
def clupi(payload):
    print(payload)
    command = bytearray()
    command.append(CLUPI_PREFIX)
    command.append(CLUPI_ADDR)
    command.extend(payload)
    command.append(0x0D)
    command.append(0x0A)
    return command
