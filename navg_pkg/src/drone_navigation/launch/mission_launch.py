import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    container = 'desafiofinal2025'
    startup_dir = '/home/rosuser/ardu_ws/Startup'
    start_sh = f'{startup_dir}/start.sh'
    projeto_yml = f'{startup_dir}/Projeto.yml'

    ensure_tools = ExecuteProcess(
        cmd=[
            'docker','exec','-u','root',container,'bash','-lc',
            (
                'set -e; '
                'command -v tmux >/dev/null 2>&1 || (apt-get update && apt-get install -y tmux); '
                'command -v tmuxinator >/dev/null 2>&1 || '
                '(apt-get update && apt-get install -y tmuxinator || '
                '(apt-get update && apt-get install -y ruby-full && gem install --no-document tmuxinator))'
            )
        ],
        shell=False,
        output='screen'
    )

    # Usa script para fornecer PTY ao tmuxinator
    run_tmuxinator = ExecuteProcess(
        cmd=[
            'docker','exec',container,'bash','-lc',
            f'export TERM=xterm-256color; cd "{startup_dir}" && script -q -f /dev/null tmuxinator start -p "{projeto_yml}"'
        ],
        shell=False,
        output='screen'
    )

    navigator = Node(
        package='drone_navigation',
        executable='navigator',
        name='navigator_node',
        output='screen'
    )

    stop_container = ExecuteProcess(
        cmd=['docker','stop',container],
        shell=False,
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg=f'Iniciando container {container}'),
        ExecuteProcess(cmd=['docker','start',container], shell=False, output='screen'),
        ExecuteProcess(cmd=['sleep','3'], shell=False),
        LogInfo(msg='Verificando ferramentas (tmux/tmuxinator)...'),
        ensure_tools,
        ExecuteProcess(cmd=['sleep','2'], shell=False),
        LogInfo(msg=f'Executando tmuxinator (Projeto.yml) com pseudo-TTY...'),
        run_tmuxinator,
        ExecuteProcess(cmd=['sleep','25'], shell=False),
        LogInfo(msg='Iniciando nó de navegação...'),
        navigator,
        RegisterEventHandler(
            OnProcessExit(
                target_action=navigator,
                on_exit=[
                    LogInfo(msg=f'Navegação terminou. Parando container {container}...'),
                    stop_container
                ]
            )
        )
    ])