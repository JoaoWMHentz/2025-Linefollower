import serial
from serial.tools import list_ports
import asyncio

class SerialService:
    @staticmethod
    def update_serial_ports(port_dropdown):
        """ Atualiza a lista de portas seriais disponíveis. """
        ports = [port.device for port in list_ports.comports()]
        port_dropdown.addItems(ports)

    @staticmethod
    def connect_serial(port_dropdown, connect_button):
        """ Conecta à porta serial selecionada. """
        port = port_dropdown.currentText()
        try:
            serial_connection = serial.Serial(port, 9600, timeout=1)
            connect_button.setText("Conectado")
            connect_button.setEnabled(False)
            return serial_connection
        except serial.SerialException:
            connect_button.setText("Erro!")
            return None

    @staticmethod
    def send_command(serial_connection, command_input, terminal):
        """ Envia um comando para o robô via serial. """
        if serial_connection and serial_connection.is_open:
            command = command_input.text()
            serial_connection.write(f"{command}\n".encode())
            terminal.append(f"Enviado: {command}")
