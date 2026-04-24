"""
ZLAC8015D driver control class.
Implements state-machine management, mode switching, and motor control.
"""

import time
from enum import IntEnum
from typing import Optional, Tuple
from .canopen_interface import (
    CANopenInterface, ObjectDictionary, ControlWord, OperationMode
)


class DriverState(IntEnum):
    """Driver state enum."""
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    QUICK_STOP_ACTIVE = 5
    FAULT_REACTION_ACTIVE = 6
    FAULT = 7
    UNKNOWN = 99


class ZLAC8015DDriver:
    """ZLAC8015D driver control class."""
    
    def __init__(self, canopen_interface: CANopenInterface):
        """
        Initialize driver.
        
        Args:
            canopen_interface: CANopen interface instance
        """
        self.can = canopen_interface
        self.current_mode = OperationMode.NO_MODE
        self.current_state = DriverState.UNKNOWN
        self.fault_code = 0
        
    def initialize(self, timeout: float = 5.0) -> bool:
        """
        Initialize driver and run state-machine startup sequence.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            True if initialization succeeds
        """
        try:
            # Send NMT start command
            self.can.send_nmt(0x01, self.can.node_id)
            time.sleep(0.1)
            
            # State machine initialization sequence: 00 -> 06 -> 07 -> 0F
            # Step 1: Shutdown (0x06)
            if not self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.SHUTDOWN):
                return False
            time.sleep(0.1)
            
            # Step 2: Switch On (0x07)
            if not self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.SWITCH_ON):
                return False
            time.sleep(0.1)
            
            # Step 3: Enable Operation (0x0F)
            if not self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.ENABLE_OPERATION):
                return False
            time.sleep(0.1)
            
            # Check state
            status = self.get_status_word()
            if status is not None:
                state = self._parse_state_from_status(status)
                self.current_state = state
                return state == DriverState.OPERATION_ENABLED
            
            return False
            
        except Exception as e:
            print(f"Driver initialization failed: {e}")
            return False
    
    def get_status_word(self) -> Optional[int]:
        """Get status word."""
        return self.can.read_u32(ObjectDictionary.STATUS_WORD, 0)
    
    def get_control_word(self) -> Optional[int]:
        """Get control word."""
        return self.can.read_u16(ObjectDictionary.CONTROL_WORD, 0)
    
    def _parse_state_from_status(self, status: int) -> DriverState:
        """Parse driver state from status word."""
        # Status word format: upper 16 bits for right motor, lower 16 bits for left motor
        left_status = status & 0xFFFF
        right_status = (status >> 16) & 0xFFFF
        
        # Use left motor status for state parsing (both sides should match)
        status_low = left_status & 0x006F  # bit0-3, bit5-6
        
        if status_low == 0x0000:
            return DriverState.NOT_READY_TO_SWITCH_ON
        elif status_low == 0x0040:  # bit6=1
            return DriverState.SWITCH_ON_DISABLED
        elif status_low == 0x0021:  # bit5=1, bit0=1
            return DriverState.READY_TO_SWITCH_ON
        elif status_low == 0x0023:  # bit5=1, bit0=1, bit1=1
            return DriverState.SWITCHED_ON
        elif status_low == 0x0027:  # bit5=1, bit0-2=1
            return DriverState.OPERATION_ENABLED
        elif status_low == 0x0007:  # bit0-2=1, bit5=0
            return DriverState.QUICK_STOP_ACTIVE
        elif (status_low & 0x004F) == 0x000F:  # bit0-3=1, bit6=0
            return DriverState.FAULT_REACTION_ACTIVE
        elif (left_status & 0x004F) == 0x0008:  # bit3=1, others are 0
            return DriverState.FAULT
        else:
            return DriverState.UNKNOWN
    
    def update_state(self):
        """Update driver state."""
        status = self.get_status_word()
        if status is not None:
            self.current_state = self._parse_state_from_status(status)
            # Check fault
            if self.current_state == DriverState.FAULT:
                self.fault_code = self.can.read_u32(ObjectDictionary.FAULT_CODE, 0) or 0
    
    def set_operation_mode(self, mode: OperationMode) -> bool:
        """
        Set operation mode.
        
        Args:
            mode: Operation mode
            
        Returns:
            True if mode change succeeds
        """
        if not self.can.write_u8(ObjectDictionary.MODE_OF_OPERATION, 0, mode.value):
            return False
        time.sleep(0.1)
        
        # Verify mode change
        current_mode = self.can.read_u8(ObjectDictionary.MODE_DISPLAY, 0)
        if current_mode == mode.value:
            self.current_mode = mode
            return True
        return False
    
    def enable_position_mode(self, 
                            max_velocity: int = 120,
                            acceleration: int = 500,
                            deceleration: int = 500) -> bool:
        """
        Enable position mode.
        
        Args:
            max_velocity: Maximum velocity (r/min, 1-1000)
            acceleration: Acceleration time (ms, 0-32767)
            deceleration: Deceleration time (ms, 0-32767)
            
        Returns:
            True if successful
        """
        if not self.set_operation_mode(OperationMode.PROFILE_POSITION):
            return False
        
        # Set parameters for left and right motors
        self.can.write_u32(ObjectDictionary.PROFILE_VELOCITY, 1, max_velocity)
        self.can.write_u32(ObjectDictionary.PROFILE_VELOCITY, 2, max_velocity)
        self.can.write_u32(ObjectDictionary.PROFILE_ACCELERATION, 1, acceleration)
        self.can.write_u32(ObjectDictionary.PROFILE_ACCELERATION, 2, acceleration)
        self.can.write_u32(ObjectDictionary.PROFILE_DECELERATION, 1, deceleration)
        self.can.write_u32(ObjectDictionary.PROFILE_DECELERATION, 2, deceleration)
        
        return True
    
    def set_target_position(self, left_position: int, right_position: int, absolute: bool = True) -> bool:
        """
        Set target position.
        
        Args:
            left_position: Left motor target position (counts)
            right_position: Right motor target position (counts)
            absolute: Whether to use absolute position
            
        Returns:
            True if successful
        """
        # Set target position
        self.can.write_i32(ObjectDictionary.TARGET_POSITION, 1, left_position)
        self.can.write_i32(ObjectDictionary.TARGET_POSITION, 2, right_position)
        
        # Trigger motion
        if absolute:
            # Absolute position: 0x0F -> 0x1F
            self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.ENABLE_OPERATION)
            time.sleep(0.01)
            self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.ABSOLUTE_POSITION_START)
        else:
            # Relative position: 0x4F -> 0x5F
            self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.ENABLE_OPERATION | 0x40)
            time.sleep(0.01)
            self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.RELATIVE_POSITION_START)
        
        return True
    
    def get_actual_position(self) -> Tuple[Optional[int], Optional[int]]:
        """Get actual position."""
        left_pos = self.can.read_i32(ObjectDictionary.ACTUAL_POSITION, 1)
        right_pos = self.can.read_i32(ObjectDictionary.ACTUAL_POSITION, 2)
        return left_pos, right_pos
    
    def enable_velocity_mode(self, 
                            sync_control: bool = False,
                            acceleration: int = 500,
                            deceleration: int = 500) -> bool:
        """
        Enable velocity mode.
        
        Args:
            sync_control: Synchronous control flag (True=synchronous, False=asynchronous)
            acceleration: Acceleration time (ms)
            deceleration: Deceleration time (ms)
            
        Returns:
            True if successful
        """
        # Set synchronous/asynchronous control flag
        sync_flag = 1 if sync_control else 0
        self.can.write_u16(ObjectDictionary.SYNC_ASYNC_FLAG, 0, sync_flag)
        
        if not self.set_operation_mode(OperationMode.PROFILE_VELOCITY):
            return False
        
        # Set acceleration and deceleration time
        self.can.write_u32(ObjectDictionary.PROFILE_ACCELERATION, 1, acceleration)
        self.can.write_u32(ObjectDictionary.PROFILE_ACCELERATION, 2, acceleration)
        self.can.write_u32(ObjectDictionary.PROFILE_DECELERATION, 1, deceleration)
        self.can.write_u32(ObjectDictionary.PROFILE_DECELERATION, 2, deceleration)
        
        return True
    
    def set_target_velocity(self, left_velocity: int, right_velocity: int, sync: bool = False) -> bool:
        """
        Set target velocity.
        
        Args:
            left_velocity: Left motor target velocity (r/min, -1000~1000)
            right_velocity: Right motor target velocity (r/min, -1000~1000)
            sync: Whether to use synchronous control
            
        Returns:
            True if successful
        """
        if sync:
            # Synchronous control: subindex 03, low 16 bits left, high 16 bits right
            combined = (right_velocity & 0xFFFF) << 16 | (left_velocity & 0xFFFF)
            self.can.write_u32(ObjectDictionary.TARGET_VELOCITY, 3, combined)
        else:
            # Asynchronous control: set left and right motors separately
            self.can.write_i32(ObjectDictionary.TARGET_VELOCITY, 1, left_velocity)
            self.can.write_i32(ObjectDictionary.TARGET_VELOCITY, 2, right_velocity)
        
        return True
    
    def get_actual_velocity(self) -> Tuple[Optional[int], Optional[int]]:
        """Get actual velocity (0.1 r/min)."""
        left_vel = self.can.read_i32(ObjectDictionary.ACTUAL_VELOCITY, 1)
        right_vel = self.can.read_i32(ObjectDictionary.ACTUAL_VELOCITY, 2)
        return left_vel, right_vel
    
    def enable_torque_mode(self,
                          sync_control: bool = False,
                          torque_slope: int = 300) -> bool:
        """
        Enable torque mode.
        
        Args:
            sync_control: Whether to use synchronous control
            torque_slope: Torque slope (mA/s, default 300)
            
        Returns:
            True if successful
        """
        # Set synchronous/asynchronous control flag
        sync_flag = 1 if sync_control else 0
        self.can.write_u16(ObjectDictionary.SYNC_ASYNC_FLAG, 0, sync_flag)
        
        if not self.set_operation_mode(OperationMode.PROFILE_TORQUE):
            return False
        
        # Set torque slope
        self.can.write_u32(ObjectDictionary.TORQUE_SLOPE, 1, torque_slope)
        self.can.write_u32(ObjectDictionary.TORQUE_SLOPE, 2, torque_slope)
        
        return True
    
    def set_target_torque(self, left_torque: int, right_torque: int, sync: bool = False) -> bool:
        """
        Set target torque.
        
        Args:
            left_torque: Left motor target torque (mA, -30000~30000)
            right_torque: Right motor target torque (mA, -30000~30000)
            sync: Whether to use synchronous control
            
        Returns:
            True if successful
        """
        if sync:
            # Synchronous control: use subindex 03
            combined = (right_torque & 0xFFFF) << 16 | (left_torque & 0xFFFF)
            self.can.write_u32(ObjectDictionary.TARGET_TORQUE, 3, combined)
        else:
            # Asynchronous control
            self.can.write_i16(ObjectDictionary.TARGET_TORQUE, 1, left_torque)
            self.can.write_i16(ObjectDictionary.TARGET_TORQUE, 2, right_torque)
        
        # Torque mode start: 0x0F -> 0x1F
        self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.ENABLE_OPERATION)
        time.sleep(0.01)
        self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.TORQUE_START)
        
        return True
    
    def get_actual_torque(self) -> Tuple[Optional[int], Optional[int]]:
        """Get actual torque (0.1 A)."""
        left_torque = self.can.read_i16(ObjectDictionary.ACTUAL_TORQUE, 1)
        right_torque = self.can.read_i16(ObjectDictionary.ACTUAL_TORQUE, 2)
        return left_torque, right_torque
    
    def stop(self):
        """Stop motors."""
        self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.DISABLE_VOLTAGE)
    
    def quick_stop(self):
        """Quick stop."""
        self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.QUICK_STOP)
    
    def clear_fault(self) -> bool:
        """Clear fault."""
        # Fault reset: set bit7 in control word
        self.can.write_u16(ObjectDictionary.CONTROL_WORD, 0, ControlWord.FAULT_RESET)
        time.sleep(0.1)
        self.fault_code = 0
        return True
    
    def save_parameters(self) -> bool:
        """Save parameters to EEPROM."""
        return self.can.write_u16(ObjectDictionary.SAVE_ALL_PARAMS, 0, 1)
