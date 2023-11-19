import json

inicial = int(input("digite o valor inicial"))
final = int(input("digite o valor final"))
x_alterna = int(input("digite o valor incial de x"))
# Carregar informações existentes do arquivo JSON (se houver)
try:
    with open('arucos_infos.json', 'r') as file:
        arucos_infos = json.load(file)
except FileNotFoundError:
    arucos_infos = []

for i in range (inicial, final):
# Operador insere informações sobre um novo marcador
    new_aruco = {}
    new_aruco['id'] = i
    new_aruco['pos_x'] = x_alterna
    new_aruco['pos_y'] = 0.6
    new_aruco['pos_z'] = 2
    new_aruco['yaw'] = -1.58
    new_aruco['size'] = 10

    x_alterna -= 1
    i += 1

# Adicionar o novo marcador à lista de informações
    arucos_infos.append(new_aruco)

# Salvar as informações atualizadas no arquivo JSON
with open('arucos_infos.json', 'w') as file:
    json.dump(arucos_infos, file, indent=4)

print("Informações do ArUco foram adicionadas e salvas no arquivo JSON.")