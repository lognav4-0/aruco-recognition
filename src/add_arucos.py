import json

# Carregar informações existentes do arquivo JSON (se houver)
try:
    with open('arucos_infos.json', 'r') as file:
        arucos_infos = json.load(file)
except FileNotFoundError:
    arucos_infos = []

# Operador insere informações sobre um novo marcador
new_aruco = {}
new_aruco['id'] = int(input("Digite o ID do novo ArUco: "))
new_aruco['pos_x'] = float(input("Digite a posição X do novo ArUco: "))
new_aruco['pos_y'] = float(input("Digite a posição Y do novo ArUco: "))
new_aruco['pos_z'] = float(input("Digite a posição Z do novo ArUco: "))
new_aruco['size'] = float(input("Digite o tamanho do novo ArUco: "))

# Adicionar o novo marcador à lista de informações
arucos_infos.append(new_aruco)

# Salvar as informações atualizadas no arquivo JSON
with open('arucos_infos.json', 'w') as file:
    json.dump(arucos_infos, file, indent=4)

print("Informações do ArUco foram adicionadas e salvas no arquivo JSON.")