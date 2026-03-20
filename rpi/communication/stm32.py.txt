from typing import Optional
import serial
from communication.link import Link
from settings import SERIAL_PORT, BAUD_RATE


class STMLink(Link):
    """Class for communicating with STM32 microcontroller over UART serial connection.

    ### RPi to STM32
    RPi sends the following commands to the STM32 in XXYYYY format:
    - XX: Detection from RPi camera (e.g., "00", "01", "02")
    - YYYY: Command to execute

    #### Movement commands
    - `FWxx`: Move forward `xx` units (e.g., "00FW05")
    - `BWxx`: Move backward `xx` units (e.g., "00BW05")
    - `FL90`: Turn left 90 degrees (e.g., "00FL90")
    - `FR90`: Turn right 90 degrees (e.g., "00FR90")
    - `FW--`: Move forward indefinitely (e.g., "00FW--")
    - `BW--`: Move backward indefinitely (e.g., "00BW--")
    - `STOP`: Stop all servos (e.g., "00STOP")

    ### STM32 to RPi
    After every command received on the STM32, an acknowledgement (string: `ACK`) must be sent back to the RPi.
    This signals to the RPi that the STM32 has completed the command, and is ready for the next command.

    """

    def __init__(self):
        """
        Constructor for STMLink.
        """
        super().__init__()
        self.serial_link = None

    def connect(self):
        """Connect to STM32 using serial UART connection, given the serial port and the baud rate"""
        try:
            self.serial_link = serial.Serial(SERIAL_PORT, BAUD_RATE)
            self.logger.info("Connected to STM32")
        except Exception as e:
            self.logger.error(f"Failed to connect to STM32: {e}")
            self.serial_link = None
            raise

    def disconnect(self):
        """Disconnect from STM32 by closing the serial link that was opened during connect()"""
        if self.serial_link is not None:
            try:
                self.serial_link.close()
                self.logger.info("Disconnected from STM32")
            except Exception as e:
                self.logger.error(f"Error disconnecting from STM32: {e}")
            finally:
                self.serial_link = None
        else:
            self.logger.info("STM32 was not connected")

    def send(self, message: str) -> None:
        """Send a message to STM32, utf-8 encoded 

        Args:
            message (str): message to send
        """
        if self.serial_link is None:
            self.logger.error("Cannot send message: STM32 not connected")
            return
        try:
            self.serial_link.write(f"{message}".encode("utf-8"))
            self.logger.debug(f"Sent to STM32: {message}")
        except Exception as e:
            self.logger.error(f"Failed to send message to STM32: {e}")

    def recv(self) -> Optional[str]:
        """Receive a message from STM32, utf-8 decoded

        Returns:
            Optional[str]: message received
        """
        if self.serial_link is None:
            self.logger.error("Cannot receive message: STM32 not connected")
            return None
        try:
            message = self.serial_link.readline().strip().decode("utf-8")
            self.logger.debug(f"Received from STM32: {message}")
            return message
        except Exception as e:
            self.logger.error(f"Failed to receive message from STM32: {e}")
            return None