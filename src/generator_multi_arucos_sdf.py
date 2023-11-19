import json
import os
import xml.etree.ElementTree as ET

caminho_arucos = os.path.expanduser('~/lognav_ws/src/freedom_vehicle/models/arucos/model.sdf')
caminho_arucos_material = os.path.expanduser('~/lognav_ws/src/freedom_vehicle/models/arucos/materials/scripts/')

with open('arucos_infos.json', 'r') as json_file:
    input_json = json.load(json_file)


target_inicial = int(input("qual o primeiro aruco que voce deseja gerar?"))
target_final = int(input("qual ultimo aruco que voce deseja gerar?"))

for i in range(target_inicial, target_final):
    target_aruco = None
    for data in input_json:
        if data["id"] == i:
            target_aruco = data
            break

    if target_aruco is None:
        print("O aruco não foi encontrado")
    else:

        # Função para criar o SDF com base nos dados do JSON
        def generate_sdf(data):
            sdf_template = f"""
        <model name='aruco_marker_{i}'>
            <static>true</static>
            <link name='link_aruco_{i}'>
            <pose>{data['pos_x']} {data['pos_y']} {data['pos_z']} 0 0 {data['yaw']}</pose>
            <inertial>
                <pose>0 0 0 0 0 0</pose>
                <mass>0.001</mass>
                <inertia>
                    <ixx>0.0</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>0.0</iyy>
                    <ixz>0.0</ixz>
                    <izz>0.0</izz>
                </inertia>
            </inertial>
            <visual name='front_visual_{i}'>
                <pose>0.00005 0 0 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.0001 {data['size']/100.0} {data['size']/100.0}</size>
                </box>
                </geometry>
                <material>
                <script>
                    <uri>model://arucos/materials/scripts/aruco_marker_{i}.material</uri>
                    <uri>model://arucos/materials/textures/{i}.png</uri>
                    <name>Marker{i}</name>
                </script>
                </material>
            </visual>
            <visual name="rear_visual">
                <pose>-0.00005 0 0 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.0001 {data['size']/100.0} {data['size']/100.0}</size>
                </box>
                </geometry>
            </visual>
            <collision name="collision">
                <pose>0 0 0 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.0001 {data['size']/100.0} {data['size']/100.0}</size>
                </box>
                </geometry>
            </collision>
            </link>
        </model>
        """
            return sdf_template

        # Lendo o conteúdo do arquivo SDF existente
        with open(caminho_arucos, 'r') as sdf_file:
            sdf_content = sdf_file.read()

            # Encontrando a posição para inserir o conteúdo gerado pelo template
            last_model_end = sdf_content.rfind("</model>")
            if last_model_end != -1:
                # Gerando o conteúdo do template
                sdf_template = generate_sdf(target_aruco)

                # Inserindo o conteúdo gerado após o último modelo
                modified_sdf_content = sdf_content[:last_model_end] + sdf_template + sdf_content[last_model_end:]

                # Salvando o SDF modificado de volta ao arquivo
                with open(caminho_arucos, 'w') as modified_sdf_file:
                    modified_sdf_file.write(modified_sdf_content)

                print(f"Conteúdo gerado pelo template adicionado após o último modelo no SDF existente.")
            else:
                print("Nenhuma tag </model> encontrada no SDF existente.")

        material_template = f"""material Marker{i}\n{{\n    technique\n    {{\n        pass\n        {{\n            texture_unit\n            {{\n                texture ../textures/{i}.png\n            }}\n        }}\n    }}\n}}"""

        material_file_path = os.path.join(caminho_arucos_material, f'aruco_marker_{i}.material')
        with open(material_file_path, 'w') as material_file:
            material_file.write(material_template)
            print(f'Arquivo {material_file_path} gerado.')
target_aruco = None
for data in input_json:
    if data["id"] == i:
        target_aruco = data
        break

if target_aruco is None:
    print("O aruco não foi encontrado")
else:

    # Função para criar o SDF com base nos dados do JSON
    def generate_sdf(data):
        sdf_template = f"""
    <model name='aruco_marker_{i}'>
        <static>true</static>
        <link name='link_aruco_{i}'>
        <pose>{data['pos_x']} {data['pos_y']} {data['pos_z']} 0 0 {data['yaw']}</pose>
        <inertial>
            <pose>0 0 0 0 0 0</pose>
            <mass>0.001</mass>
            <inertia>
                <ixx>0.0</ixx>
                <ixy>0.0</ixy>
                <ixz>0.0</ixz>
                <iyy>0.0</iyy>
                <ixz>0.0</ixz>
                <izz>0.0</izz>
            </inertia>
        </inertial>
        <visual name='front_visual_{i}'>
            <pose>0.00005 0 0 0 0 0</pose>
            <geometry>
            <box>
                <size>0.0001 {data['size']/100.0} {data['size']/100.0}</size>
            </box>
            </geometry>
            <material>
            <script>
                <uri>model://arucos/materials/scripts/aruco_marker_{i}.material</uri>
                <uri>model://arucos/materials/textures/{i}.png</uri>
                <name>Marker{i}</name>
            </script>
            </material>
        </visual>
        <visual name="rear_visual">
            <pose>-0.00005 0 0 0 0 0</pose>
            <geometry>
            <box>
                <size>0.0001 {data['size']/100.0} {data['size']/100.0}</size>
            </box>
            </geometry>
        </visual>
        <collision name="collision">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
            <box>
                <size>0.0001 {data['size']/100.0} {data['size']/100.0}</size>
            </box>
            </geometry>
        </collision>
        </link>
    </model>
    """
        return sdf_template

    # Lendo o conteúdo do arquivo SDF existente
    with open(caminho_arucos, 'r') as sdf_file:
        sdf_content = sdf_file.read()

        # Encontrando a posição para inserir o conteúdo gerado pelo template
        last_model_end = sdf_content.rfind("</model>")
        if last_model_end != -1:
            # Gerando o conteúdo do template
            sdf_template = generate_sdf(target_aruco)

            # Inserindo o conteúdo gerado após o último modelo
            modified_sdf_content = sdf_content[:last_model_end] + sdf_template + sdf_content[last_model_end:]

            # Salvando o SDF modificado de volta ao arquivo
            with open(caminho_arucos, 'w') as modified_sdf_file:
                modified_sdf_file.write(modified_sdf_content)

            print(f"Conteúdo gerado pelo template adicionado após o último modelo no SDF existente.")
        else:
            print("Nenhuma tag </model> encontrada no SDF existente.")

    material_template = f"""material Marker{i}\n{{\n    technique\n    {{\n        pass\n        {{\n            texture_unit\n            {{\n                texture ../textures/{i}.png\n            }}\n        }}\n    }}\n}}"""

    material_file_path = os.path.join(caminho_arucos_material, f'aruco_marker_{i}.material')
    with open(material_file_path, 'w') as material_file:
        material_file.write(material_template)
        print(f'Arquivo {material_file_path} gerado.')
    i += 1