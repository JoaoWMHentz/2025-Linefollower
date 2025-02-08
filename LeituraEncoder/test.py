import serial

# Configuração da porta serial
porta = 'COM11'  # Nome da porta serial no Windows
baud_rate = 115200  # Taxa de transmissão

try:
    # Abrir a porta serial
    with serial.Serial(porta, baud_rate, timeout=1) as ser, open("dados.txt", "a") as arquivo:
        print(f"Lendo dados da {porta} e salvando em dados_serial.txt... (Pressione Ctrl+C para parar)")
        
        while True:
            linha = ser.readline().decode('utf-8').strip()
            if linha:
                print(linha)
                arquivo.write(linha + "\n")
                arquivo.flush()  # Garante que os dados sejam escritos imediatamente no arquivo

except serial.SerialException as e:
    print(f"Erro ao abrir a porta serial: {e}")

except KeyboardInterrupt:
    print("\nLeitura interrompida pelo usuário.")
