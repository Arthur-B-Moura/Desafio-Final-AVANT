# Desafio-Final-AVANT
Repositório para gerenciamento dos códigos para o Desafio Final da AVANT - Grupo 2 Elt

> [!NOTE]
> Em construção


## Módulo de controle do Gancho

Há dois pacotes referentes ao módulo do gancho:
- `gancho_pkg` trata-se do pacote principal, com todas as funcionalidades, desde comunicação em tópicos ROS2 até o controle físico de elementos como o relé pela *Jetson Nano*. Para permitir testes em ambientes onde a Jetson não está presente, há o módulo seguinte.
  
- `gancho_pkg_tester` é o módulo para teste da comunicação do algoritmo para ambientes em que a Jetson não está presente. A comunicação dele ocorre nos mesmos tópicos, pelas mesmas mensagens, mas sem a inclusão de funções da *Jetson.DPIO*.



### Compilação e execução dos Nodes

Para compilar e rodar o algoritmo, digite, a partir do diretório do workspace ROS2, os seguintes comandos:
```
$ colcon build --packages-select gancho_pkg
$ source ~/.bashrc
$ ros2 run gancho_pkg gancho_node
```

Dentro do *pkg* do módulo do gancho existe um arquibo `tester.py`, que permite testar a comunicação e funcionamento dos algoritmos de forma individual, sem a necessidade de executar outros algoritmos ou compilar outros *pkgs*. Para executá-lo, basta, após compilar o pacote, rodar em outro terminal o node ros2 de teste:
```
$ ros2 run gancho_pkg tester_node
```
Ao digitar qualquer input neste terminal, uma mensagem será enviada, simulando a mensagem de <ins>'drone centralizado'</ins>, o que permitirá o teste das funcionalidades do algoritmo principal do gancho.


Caso deseje executar o *pkg* de teste, basta apenas trocar `gancho_pkg` por `gancho_pkg_tester`. O nome dos nós é o mesmo para ambos. 



### Tópicos e mensagens utilizados

- Para a mensagem de <ins>'drone centralizado'</ins>, espera-se uma mensagem `String()` enviada no tópico `'/gancho/posicao_drone'`. O texto *msg.data* esperado é **"Drone centralizado"**.
- O algoritmo envia mensagens de <ins>status</ins> a cada 2s, também do tipo `String()`, no tópico `'/gancho/status'`, de forma que *msg.data* será:
   
  -> **"waiting"** - node em execução, ainda não recebeu mensagem de drone centralizado.
  -> **"preparing"** - recebeu mensagem de drone centralizado, mas ainda não realizou as operações para soltar o gancho.
  -> **"released"** - gancho já foi solto, procedimento do algorítmo foi finalizado.
