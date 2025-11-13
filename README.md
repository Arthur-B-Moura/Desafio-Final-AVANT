# Desafio Final AVANT - Missão "Hang the Hook"

Projeto final do trainee AVANT 2025 - Sistema autônomo para detecção de linha, identificação de alvo e liberação de gancho usando ROS 2.
Grupo 2 Eletrônica

> [!NOTE]
> Em construção

---

## Descrição do Sistema

O sistema é composto por três módulos principais:

1. **Visão Computacional** (`drone_vision`) - Detecta linha azul e mangueira vermelha
2. **Navegação** (`drone_navigation`) - Controla o movimento do drone
3. **Controle do Gancho** (`gancho_pkg`) - Aciona o relé para soltar o gancho

### Fluxo de Operação

```
[Visão] → detecta linha azul → publica posição
    ↓
[Navegação] → segue linha → busca mangueira vermelha
    ↓
[Visão] → detecta vermelho → publica "centralizado"
    ↓
[Navegação] → alinha drone com alvo
    ↓
[Gancho] → recebe sinal → libera gancho
```

---

## Instalação e Compilação

### Pré-requisitos

- ROS 2 (Humble ou superior)
- OpenCV
- Python 3
- Docker (para simulação)

### Compilação dos Pacotes

A partir do diretório do workspace:

```bash
# Compilar todos os pacotes necessários
colcon build --packages-select gancho_pkg_tester drone_navigation drone_vision

# Recarregar o ambiente
source ~/.bashrc

# OU source install/setup.bash se não configurou no .bashrc
source install/setup.bash
```

---

## Execução

### Executar Simulação Completa

Para rodar o sistema completo (visão + navegação + gancho):

```bash
ros2 launch drone_navigation solution.launch.py
```

### Executar Módulos Individualmente

#### 1. Módulo de Visão Computacional

```bash
# Executar detecção de linha e mangueira
ros2 launch drone_vision vision.launch.py

# Visualizar imagem de debug (opcional, em outro terminal)
ros2 run rqt_image_view rqt_image_view
# Selecione o tópico: /vision/debug_image
```

#### 2. Módulo de Navegação

```bash
ros2 run drone_navigation navigation_node
```

#### 3. Módulo do Gancho

```bash
# Versão com controle real da Jetson (para hardware)
ros2 run gancho_pkg gancho_node

# Versão de teste (sem Jetson)
ros2 run gancho_pkg_tester gancho_node
```

---

## Tópicos ROS 2 e Mensagens

### Módulo de Visão Computacional

**Publicações:**
- `/vision/line_position` - `std_msgs/Int32MultiArray`
  - `[cx, cy]` - Coordenadas do centro da linha azul em pixels
  - `[-1, -1]` - Quando linha não detectada
- `/vision/red_detected` - `std_msgs/Bool`
  - `True` - Mangueira vermelha detectada
  - `False` - Mangueira não detectada
- `/vision/state` - `std_msgs/String`
  - `"seguindo"` - Seguindo linha azul
  - `"vermelho_detectado"` - Mangueira vermelha encontrada
  - `"perdido"` - Nenhum alvo detectado
- `/vision/debug_image` - `sensor_msgs/Image`
  - Imagem anotada com detecções (para debug visual)

**Subscrições:**
- `/camera/image_raw` - `sensor_msgs/Image`
  - Feed da câmera do drone

### Módulo do Gancho

**Subscrições:**
- `/gancho/posicao_drone` - `std_msgs/String`
  - Mensagem esperada: `msg.data = "Drone centralizado"`

**Publicações:**
- `/gancho/status` - `std_msgs/String`
  - `"waiting"` - Aguardando sinal de centralização
  - `"preparing"` - Preparando para soltar gancho
  - `"released"` - Gancho liberado com sucesso

---

## Testes

### Testar Módulo de Visão

```bash
# Terminal 1: Executar nó de visão
ros2 launch drone_vision vision.launch.py

# Terminal 2: Monitorar detecções
ros2 topic echo /vision/state

# Terminal 3: Ver posição da linha
ros2 topic echo /vision/line_position
```

### Testar Comunicação do Gancho

Existe um node de teste (`tester_node`) que simula a mensagem de "drone centralizado":

```bash
# Terminal 1: Executar gancho em modo teste
colcon build --packages-select gancho_pkg_tester
source install/setup.bash
ros2 run gancho_pkg_tester gancho_node

# Terminal 2: Monitorar status
ros2 topic echo /gancho/status

# Terminal 3: Executar testador
ros2 run gancho_pkg_tester tester_node
# Digite qualquer tecla + Enter para simular "Drone centralizado"
```

### Verificar Comunicação Entre Módulos

```bash
# Listar todos os tópicos ativos
ros2 topic list

# Ver informações de um tópico específico
ros2 topic info /vision/line_position

# Ver taxa de publicação
ros2 topic hz /vision/state

# Verificar conexões
ros2 node list
ros2 node info /line_detector_node
```

---

## Configuração e Calibração

### Ajustar Parâmetros de Visão

Edite o arquivo `drone_vision/launch/vision.launch.py` para ajustar:

#### Cores HSV
```python
# Azul (linha no chão)
'azul_h_min': 100,    # Matiz mínimo
'azul_s_min': 150,    # Saturação mínima
'azul_v_min': 50,     # Valor mínimo
'azul_h_max': 140,    # Matiz máximo
'azul_s_max': 255,    # Saturação máxima
'azul_v_max': 255,    # Valor máximo

# Vermelho (mangueira) - Faixa 1
'red1_h_min': 0,
'red1_h_max': 10,

# Vermelho (mangueira) - Faixa 2
'red2_h_min': 170,
'red2_h_max': 180,
```

#### Processamento
```python
'min_area': 300,                # Área mínima para detecção (pixels²)
'smoothing_window': 5,          # Janela de suavização
'roi_vertical_fraction': 0.4,   # Fração inferior da imagem analisada
```

### Modo Calibração

Para ajustar os valores HSV visualmente:

```bash
ros2 launch drone_vision vision.launch.py calibration_mode:=true
```

Isso abrirá janelas OpenCV mostrando as máscaras de cor em tempo real.

---

## Ambiente Docker (Simulação)

### Primeira Execução

```bash
# Baixar imagem
docker pull joao0607/desafiofinal2025

# Executar container
docker run -it \
  --name desafiofinal2025 \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --net=host \
  joao0607/desafiofinal2025
```

### Execuções Subsequentes

```bash
# Iniciar container
docker start desafiofinal2025

# Acessar terminal
docker exec -it desafiofinal2025 bash

# Dentro do container, iniciar simulação
cd ~/ardu_ws/Startup
./start_simulation.sh  # ou o comando específico do tmux
```

### Remover Container Antigo (se necessário)

```bash
docker ps -a                        # Listar containers
docker stop aula_navegacao          # Parar container antigo
docker rm aula_navegacao            # Remover container
docker rmi aula_navegacao           # Remover imagem
```

---

## Estrutura de Pacotes

```
Desafio-Final-AVANT/
├── drone_vision/              # Visão computacional
│   ├── drone_vision/
│   │   ├── __init__.py
│   │   └── line_detector.py   # Nó de detecção
│   ├── launch/
│   │   └── vision.launch.py   # Launch file
│   ├── resource/
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
│
├── gancho_pkg/                # Controle do gancho (Jetson)
│   ├── gancho_pkg/
│   │   ├── __init__.py
│   │   ├── gancho.py          # Nó principal
│   │   └── tester.py          # Nó de teste
│   ├── resource/
│   ├── test/
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
│
├── gancho_pkg_tester/         # Teste sem Jetson
│   └── [mesma estrutura]
│
├── navg_pkg/                  # Navegação do drone
│   ├── src/
│   │   └── drone_navigation/
│   ├── build/
│   └── install/
│
└── README.md
```

---

## Troubleshooting

### Erro: "Package not found"

```bash
# Verificar se pacote foi compilado
colcon list

# Recompilar
colcon build --packages-select <nome_do_pacote>
source install/setup.bash
```

### Erro: "No module named cv2"

```bash
pip3 install opencv-python opencv-contrib-python
```

### Erro: "Topic not published"

```bash
# Verificar se nó está rodando
ros2 node list

# Verificar se tópico existe
ros2 topic list

# Ver informações do tópico
ros2 topic info /vision/line_position
```

### Visão não detecta cores

1. Execute em modo calibração:
   ```bash
   ros2 launch drone_vision vision.launch.py calibration_mode:=true
   ```

2. Ajuste os valores HSV no `vision.launch.py`

3. Verifique iluminação do ambiente (muito importante!)

### Erros MAVLink na simulação

Os warnings de MAVLink (`DeviceError:url:UDP separator not found`) são **normais** e não impedem o funcionamento. São apenas avisos de reconexão do protocolo.

---

## 👥 Equipe
- Arthur Bertolini
- Gabriel Ribeiro
- Mateus Gontijo