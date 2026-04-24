"""
CANopen communication interface abstraction.
Provides basic CANopen features such as SDO read/write and NMT control.
"""

import struct
import time
import threading
from enum import IntEnum
from typing import Optional, Tuple


class OperationMode(IntEnum):
    """Operation mode enum."""
    NO_MODE = 0
    PROFILE_POSITION = 1
    PROFILE_VELOCITY = 3
    PROFILE_TORQUE = 4


class ControlWord:
    """Control word definitions."""
    SHUTDOWN = 0x06
    SWITCH_ON = 0x07
    DISABLE_VOLTAGE = 0x00
    QUICK_STOP = 0x02
    DISABLE_OPERATION = 0x07
    ENABLE_OPERATION = 0x0F
    FAULT_RESET = 0x80
    
    # Position mode control words
    ABSOLUTE_POSITION_START = 0x1F  # 0x0F -> 0x1F
    RELATIVE_POSITION_START = 0x5F  # 0x4F -> 0x5F
    
    # Torque mode control words
    TORQUE_START = 0x1F  # 0x0F -> 0x1F


class ObjectDictionary:
    """Object dictionary index definitions."""
    # Communication parameters
    ERROR_REGISTER = 0x1001
    HEARTBEAT_PRODUCER = 0x1017
    SAVE_ALL_PARAMS = 0x2010
    
    # Control parameters
    CONTROL_WORD = 0x6040
    STATUS_WORD = 0x6041
    MODE_OF_OPERATION = 0x6060
    MODE_DISPLAY = 0x6061
    FAULT_CODE = 0x603F
    
    # Position mode parameters
    TARGET_POSITION = 0x607A
    ACTUAL_POSITION = 0x6064
    PROFILE_VELOCITY = 0x6081
    PROFILE_ACCELERATION = 0x6083
    PROFILE_DECELERATION = 0x6084
    
    # Velocity mode parameters
    TARGET_VELOCITY = 0x60FF
    ACTUAL_VELOCITY = 0x606C
    
    # Torque mode parameters
    TARGET_TORQUE = 0x6071
    ACTUAL_TORQUE = 0x6077
    TORQUE_SLOPE = 0x6087
    
    # Custom parameter
    SYNC_ASYNC_FLAG = 0x200F


class CANopenInterface:
    """
    Base class for CANopen communication interfaces.
    Concrete implementations should use a real CAN library
    (for example python-can or canopen).
    """
    
    def __init__(self, node_id: int = 1, can_interface: str = "can0"):
        """
        Initialize CANopen interface.
        
        Args:
            node_id: Node ID (1-127)
            can_interface: CAN interface name
        """
        self.node_id = node_id
        self.can_interface = can_interface
        self.tx_sdo_cobid = 0x600 + node_id
        self.rx_sdo_cobid = 0x580 + node_id
        self.heartbeat_cobid = 0x700 + node_id
        
        # State
        self.is_connected = False
        self.heartbeat_received = False
        
    def connect(self) -> bool:
        """
        Connect to CAN bus.
        
        Returns:
            True if connected successfully
        """
        # Implement with the selected CAN library
        # Example with python-can:
        # import can
        # self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
        raise NotImplementedError("connect() must be implemented in a subclass")
    
    def disconnect(self):
        """Disconnect from CAN bus."""
        # Implement with the selected CAN library
        raise NotImplementedError("disconnect() must be implemented in a subclass")
    
    def send_nmt(self, command: int, node_id: Optional[int] = None):
        """
        Send NMT command.
        
        Args:
            command: NMT command (0x01=start, 0x02=stop, 0x80=pre-op, 0x81=reset app, 0x82=reset comm)
            node_id: Target node ID, None means broadcast (0)
        """
        target_id = node_id if node_id is not None else 0
        data = bytes([command, target_id])
        # Implement CAN frame send in subclass
        raise NotImplementedError("send_nmt() must be implemented in a subclass")
    
    def sdo_write(self, index: int, subindex: int, data: bytes, data_type: str = "auto") -> bool:
        """
        Write object dictionary entry via SDO.
        
        Args:
            index: Object dictionary index
            subindex: Subindex
            data: Data bytes
            data_type: Data type ("auto", "u8", "i8", "u16", "i16", "u32", "i32")
            
        Returns:
            True if write succeeds
        """
        # Select SDO command byte based on payload length
        data_len = len(data)
        if data_type == "auto":
            if data_len == 1:
                cmd = 0x2F  # 1 byte
            elif data_len == 2:
                cmd = 0x2B  # 2 bytes
            elif data_len == 3:
                cmd = 0x27  # 3 bytes
            elif data_len == 4:
                cmd = 0x23  # 4 bytes
            else:
                raise ValueError(f"Unsupported data length: {data_len}")
        else:
            # Select command byte from data_type
            type_map = {"u8": 0x2F, "i8": 0x2F, "u16": 0x2B, "i16": 0x2B,
                       "u32": 0x23, "i32": 0x23}
            cmd = type_map.get(data_type, 0x23)
        
        # Build SDO request frame
        index_bytes = struct.pack('<H', index)
        sdo_data = bytes([cmd]) + index_bytes + bytes([subindex]) + data
        sdo_data = sdo_data + bytes(8 - len(sdo_data))  # Pad to 8 bytes
        
        # Send SDO request and wait for response
        # Implement CAN TX/RX in subclass
        raise NotImplementedError("sdo_write() must be implemented in a subclass")
    
    def sdo_read(self, index: int, subindex: int, data_type: str = "u32") -> Optional[bytes]:
        """
        Read object dictionary entry via SDO.
        
        Args:
            index: Object dictionary index
            subindex: Subindex
            data_type: Data type ("u8", "i8", "u16", "i16", "u32", "i32")
            
        Returns:
            Read data, or None on failure
        """
        cmd = 0x40  # Read command
        index_bytes = struct.pack('<H', index)
        sdo_data = bytes([cmd]) + index_bytes + bytes([subindex]) + bytes(4)
        
        # Send SDO request and wait for response
        # Parse response frame and extract data in subclass
        raise NotImplementedError("sdo_read() must be implemented in a subclass")
    
    def write_u8(self, index: int, subindex: int, value: int) -> bool:
        """Write unsigned 8-bit integer."""
        return self.sdo_write(index, subindex, struct.pack('<B', value), "u8")
    
    def write_u16(self, index: int, subindex: int, value: int) -> bool:
        """Write unsigned 16-bit integer."""
        return self.sdo_write(index, subindex, struct.pack('<H', value), "u16")
    
    def write_i16(self, index: int, subindex: int, value: int) -> bool:
        """Write signed 16-bit integer."""
        return self.sdo_write(index, subindex, struct.pack('<h', value), "i16")
    
    def write_u32(self, index: int, subindex: int, value: int) -> bool:
        """Write unsigned 32-bit integer."""
        return self.sdo_write(index, subindex, struct.pack('<I', value), "u32")
    
    def write_i32(self, index: int, subindex: int, value: int) -> bool:
        """Write signed 32-bit integer."""
        return self.sdo_write(index, subindex, struct.pack('<i', value), "i32")
    
    def read_u8(self, index: int, subindex: int) -> Optional[int]:
        """Read unsigned 8-bit integer."""
        data = self.sdo_read(index, subindex, "u8")
        return struct.unpack('<B', data)[0] if data and len(data) >= 1 else None
    
    def read_u16(self, index: int, subindex: int) -> Optional[int]:
        """Read unsigned 16-bit integer."""
        data = self.sdo_read(index, subindex, "u16")
        return struct.unpack('<H', data[:2])[0] if data and len(data) >= 2 else None
    
    def read_i16(self, index: int, subindex: int) -> Optional[int]:
        """Read signed 16-bit integer."""
        data = self.sdo_read(index, subindex, "i16")
        return struct.unpack('<h', data[:2])[0] if data and len(data) >= 2 else None
    
    def read_u32(self, index: int, subindex: int) -> Optional[int]:
        """Read unsigned 32-bit integer."""
        data = self.sdo_read(index, subindex, "u32")
        return struct.unpack('<I', data[:4])[0] if data and len(data) >= 4 else None
    
    def read_i32(self, index: int, subindex: int) -> Optional[int]:
        """Read signed 32-bit integer."""
        data = self.sdo_read(index, subindex, "i32")
        return struct.unpack('<i', data[:4])[0] if data and len(data) >= 4 else None


class MockCANopenInterface(CANopenInterface):
    """
    Mock CANopen implementation for testing.
    Does not perform real CAN communication; only simulates responses.
    """
    
    def __init__(self, node_id: int = 1, can_interface: str = "can0"):
        super().__init__(node_id, can_interface)
        self.mock_registers = {}
        self.is_connected = True
        # Initialize status word: both motors in SWITCH_ON_DISABLED state (0x0040)
        # Status word format: upper 16 bits for right motor, lower 16 bits for left motor
        # 0x0040 = bit6=1 (SWITCH_ON_DISABLED)
        initial_status = 0x00400040  # Both motors are 0x0040
        self.mock_registers[(ObjectDictionary.STATUS_WORD, 0)] = struct.pack('<I', initial_status)
        
    def connect(self) -> bool:
        self.is_connected = True
        return True
    
    def disconnect(self):
        self.is_connected = False
    
    def send_nmt(self, command: int, node_id: Optional[int] = None):
        # Simulate NMT command: after start command, state becomes READY_TO_SWITCH_ON
        if command == 0x01:  # NMT start command
            # State changes to READY_TO_SWITCH_ON (0x0021: bit5=1, bit0=1)
            status = 0x00210021  # Both motors are 0x0021
            self.mock_registers[(ObjectDictionary.STATUS_WORD, 0)] = struct.pack('<I', status)
    
    def sdo_write(self, index: int, subindex: int, data: bytes, data_type: str = "auto") -> bool:
        key = (index, subindex)
        self.mock_registers[key] = data
        
        # If control word is written, update status word accordingly
        if index == ObjectDictionary.CONTROL_WORD and subindex == 0:
            control_word = struct.unpack('<H', data[:2])[0] if len(data) >= 2 else 0
            
            # Read current status word
            current_status_data = self.mock_registers.get((ObjectDictionary.STATUS_WORD, 0), struct.pack('<I', 0x00400040))
            current_status = struct.unpack('<I', current_status_data)[0]
            left_status = current_status & 0xFFFF
            right_status = (current_status >> 16) & 0xFFFF
            
            # Update state based on control word (simplified state machine)
            # 0x06 (SHUTDOWN) -> READY_TO_SWITCH_ON (0x0021)
            # 0x07 (SWITCH_ON) -> SWITCHED_ON (0x0023)
            # 0x0F (ENABLE_OPERATION) -> OPERATION_ENABLED (0x0027)
            if control_word == 0x06:  # SHUTDOWN
                new_status = 0x00210021  # READY_TO_SWITCH_ON
            elif control_word == 0x07:  # SWITCH_ON
                new_status = 0x00230023  # SWITCHED_ON
            elif control_word == 0x0F:  # ENABLE_OPERATION
                new_status = 0x00270027  # OPERATION_ENABLED
            elif control_word == 0x00:  # DISABLE_VOLTAGE
                new_status = 0x00400040  # SWITCH_ON_DISABLED
            elif control_word == 0x1F:  # ABSOLUTE_POSITION_START or TORQUE_START
                # Keep OPERATION_ENABLED state
                new_status = 0x00270027
            elif control_word == 0x5F:  # RELATIVE_POSITION_START
                # Keep OPERATION_ENABLED state
                new_status = 0x00270027
            else:
                # Other control words keep current state
                new_status = current_status
            
            self.mock_registers[(ObjectDictionary.STATUS_WORD, 0)] = struct.pack('<I', new_status)
        
        return True
    
    def sdo_read(self, index: int, subindex: int, data_type: str = "u32") -> Optional[bytes]:
        key = (index, subindex)
        if key in self.mock_registers:
            data = self.mock_registers[key]
            # Pad to 4 bytes
            return data + bytes(4 - len(data))
        
        # For status word, return default SWITCH_ON_DISABLED
        if index == ObjectDictionary.STATUS_WORD and subindex == 0:
            return struct.pack('<I', 0x00400040)
        
        # Return default value for other registers
        return bytes([0, 0, 0, 0])


class SocketCANopenInterface(CANopenInterface):
    """
    SocketCAN implementation using python-can.
    Used to communicate with real hardware.
    """
    
    def __init__(self, node_id: int = 1, can_interface: str = "can0"):
        super().__init__(node_id, can_interface)
        self.bus = None
        self.sdo_timeout = 1.0  # SDO timeout (seconds)
        self.response_event = threading.Event()
        self.response_data = None
        
    def connect(self) -> bool:
        """Connect to CAN bus."""
        try:
            import can
            self.bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=500000
            )
            self.is_connected = True
            
            # Start receive thread
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            return True
        except ImportError:
            raise ImportError("python-can is required: pip install python-can")
        except Exception as e:
            print(f"Failed to connect CAN bus: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from CAN bus."""
        self.is_connected = False
        if self.bus:
            self.bus.shutdown()
            self.bus = None
    
    def _receive_loop(self):
        """CAN message receive loop."""
        while self.is_connected and self.bus:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                
                # Handle SDO response
                if msg.arbitration_id == self.rx_sdo_cobid:
                    self.response_data = msg.data
                    self.response_event.set()
            except Exception as e:
                if self.is_connected:
                    print(f"CAN receive error: {e}")
    
    def send_nmt(self, command: int, node_id: Optional[int] = None):
        """Send NMT command."""
        if not self.bus:
            return
        
        try:
            import can
            target_id = node_id if node_id is not None else 0
            data = bytes([command, target_id])
            
            msg = can.Message(
            arbitration_id=0x000,  # NMT COB-ID
                data=data,
                is_extended_id=False
            )
            
            self.bus.send(msg)
        except Exception as e:
            print(f"Failed to send NMT command: {e}")
    
    def sdo_write(self, index: int, subindex: int, data: bytes, data_type: str = "auto") -> bool:
        """Write object dictionary entry via SDO."""
        if not self.bus:
            return False
        
        # Select SDO command byte based on payload length
        data_len = len(data)
        if data_type == "auto":
            if data_len == 1:
                cmd = 0x2F  # 1 byte
            elif data_len == 2:
                cmd = 0x2B  # 2 bytes
            elif data_len == 3:
                cmd = 0x27  # 3 bytes
            elif data_len == 4:
                cmd = 0x23  # 4 bytes
            else:
                raise ValueError(f"Unsupported data length: {data_len}")
        else:
            type_map = {"u8": 0x2F, "i8": 0x2F, "u16": 0x2B, "i16": 0x2B,
                       "u32": 0x23, "i32": 0x23}
            cmd = type_map.get(data_type, 0x23)
        
        # Build SDO request frame
        index_bytes = struct.pack('<H', index)
        sdo_data = bytes([cmd]) + index_bytes + bytes([subindex]) + data
        sdo_data = sdo_data + bytes(8 - len(sdo_data))  # Pad to 8 bytes
        
        # Send SDO request
        try:
            import can
            msg = can.Message(
                arbitration_id=self.tx_sdo_cobid,
                data=sdo_data,
                is_extended_id=False
            )
            
            self.response_event.clear()
            self.response_data = None
            self.bus.send(msg)
            
            # Wait for response
            if self.response_event.wait(timeout=self.sdo_timeout):
                # Check whether response indicates success
                if self.response_data and len(self.response_data) >= 1:
                    response_cmd = self.response_data[0]
                    # 0x60 means write success
                    if response_cmd == 0x60:
                        return True
                    # 0x80 means error
                    elif (response_cmd & 0xE0) == 0x80:
                        error_code = struct.unpack('<I', self.response_data[4:8])[0]
                        print(f"SDO write error: index=0x{index:04X}, subindex=0x{subindex:02X}, error=0x{error_code:08X}")
                        return False
            
            # Timeout or invalid response
            return False
        except Exception as e:
            print(f"Failed to send SDO write request: {e}")
            return False
    
    def sdo_read(self, index: int, subindex: int, data_type: str = "u32") -> Optional[bytes]:
        """Read object dictionary entry via SDO."""
        if not self.bus:
            return None
        
        cmd = 0x40  # Read command
        index_bytes = struct.pack('<H', index)
        sdo_data = bytes([cmd]) + index_bytes + bytes([subindex]) + bytes(4)
        
        # Send SDO request
        try:
            import can
            msg = can.Message(
                arbitration_id=self.tx_sdo_cobid,
                data=sdo_data,
                is_extended_id=False
            )
            
            self.response_event.clear()
            self.response_data = None
            self.bus.send(msg)
            
            # Wait for response
            if self.response_event.wait(timeout=self.sdo_timeout):
                if self.response_data and len(self.response_data) >= 1:
                    response_cmd = self.response_data[0]
                    # 0x4B/0x4F/0x43/0x47 indicate successful read
                    if (response_cmd & 0xE0) == 0x40:
                        # Extract payload based on command byte
                        if response_cmd == 0x4F:  # 1 byte
                            data = self.response_data[4:5]
                        elif response_cmd == 0x4B:  # 2 bytes
                            data = self.response_data[4:6]
                        elif response_cmd == 0x47:  # 3 bytes
                            data = self.response_data[4:7]
                        elif response_cmd == 0x43:  # 4 bytes
                            data = self.response_data[4:8]
                        else:
                            return None
                        return data
                    # 0x80 means error
                    elif (response_cmd & 0xE0) == 0x80:
                        error_code = struct.unpack('<I', self.response_data[4:8])[0]
                        print(f"SDO read error: index=0x{index:04X}, subindex=0x{subindex:02X}, error=0x{error_code:08X}")
                        return None
            
            # Timeout or invalid response
            return None
        except Exception as e:
            print(f"Failed to send SDO read request: {e}")
            return None
