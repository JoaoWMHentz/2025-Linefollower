import asyncio
import websockets
import json
import random

SERVER_URL = "wss://neon-drift-server.homelab.vini.center"

# Função que cria uma conexão WebSocket e manda uma mensagem com dados aleatórios
async def send_test_message(client_id):
    async with websockets.connect(SERVER_URL) as websocket:
        print(f"[{client_id}] Conectado!")

        # Exemplo de payload (ajuste conforme o que o servidor espera)
        message = {
            "type": "test_message",
            "client_id": client_id,
            "username": f"Player_{random.randint(1000, 9999)}",
            "action": random.choice(["move", "shoot", "boost"]),
            "x": random.uniform(0, 100),
            "y": random.uniform(0, 100)
        }

        # Envia a mensagem
        await websocket.send(json.dumps(message))
        print(f"[{client_id}] Mensagem enviada: {message}")

        # Aguarda uma possível resposta
        try:
            response = await websocket.recv()
            print(f"[{client_id}] Resposta: {response}")
        except websockets.exceptions.ConnectionClosedOK:
            print(f"[{client_id}] Conexão encerrada pelo servidor.")

# Lança várias conexões em paralelo
async def main():
    tasks = []
    for i in range(100):  # Aqui define quantos clientes simultâneos você quer
        tasks.append(send_test_message(f"client_{i+1}"))
    
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
