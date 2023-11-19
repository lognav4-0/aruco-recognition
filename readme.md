# Aruco Recognition - LOGNAV 4.0

Este pacote foi criado para a detecção de arucos no ambiente simulado e real, além da geração dos mesmos no ambiente simulado utilizando sdf.

### 📋 Pré-requisitos

O pacote foi criado utilizando [ros2 humble](https://docs.ros.org/en/humble/Installation.html).

### 🔧 Instalação

Primeiramente deverá ser instalado o pacote Freedom_vehicle, onde temos a simulação:

```
https://github.com/lognav4-0/freedom_vehicle
```

Após ter o ambiente simulado devidamente instalado e o .bash devidamente atualizado, deverá clonar o pacote aruco_recognition:

```
git clone https://github.com/lognav4-0/aruco_recognition.git
```


## ⚙️ Adicionando Arucos no mundo Simulado.

Para adicionar os arucos no ambiente simulado, primeiramente deve-se entrar na pasta src.

Após, deverá rodar o código "aruco_generator2.py", que será responsavel por gerar as imagens do tipo de ArUco selecionado.

Após ter as imagens geradas, caso queira gerar um único aruco, deverá rodar o código "add_arucos.py", que será responsavel de adicionar os arucos para o arquivo "arucos_infos.json".

Logo após ter o arquivo .json completo, deverá rodar o código generator_sdf, que será responsavel por adicionar os ArUcos a um sdf no ambiente simulado.

OBS: Caso desejar gerar mais de um ArUco, podem ser utilizados os códigos "generator_multi_arucos_json.py" e "generator_multi_arucos_sdf.py" respectivamente. Neles, será possivel escolher a partir de qual aruco e até que o usuário deseja gerar.

OBS: Após gerar algum aruco novo, o pacote deverá ser buildado novamente com os seguintes comandos:

```
cd ~/lognav_ws ##ou o nome do seu workspace

```

```
colcon build

```

```
source install/setup.bash
```



### 🔩 Erros

Em caso de erros, pode ser levado em consideração as seguintes situações:

* 1° - Entrar na pasta src antes de rodar os códigos de geração de ArUcos.
* 2° - Caso esteja com erro para atualizar o arquivo .json, ele pode ser removido e gerado novamente.
* 3° - Caso não esteja conseguindo gerar as imagens, pode-se gerar a pasta "arucos" dentro do pacote freedom_vehicle manualmente.

### ⌨️ Reconhecendo ArUcos.

Para o reconhecimento dos arucos no ambiente simulado, deve-se rodar o seguinte código:

```
ros2 run aruco_recognition aruco-recog.py
```

Para o reconhecimento dos arucos no ambiente real, deve-se rodar o seguinte código:

```
ros2 run aruco_recognition aruco-recog-camera.py
```

