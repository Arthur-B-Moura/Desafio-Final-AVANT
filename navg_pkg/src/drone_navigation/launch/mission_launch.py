import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    container_name = 'desafiofinal2025'
    startup_dir = '/home/rosuser/ardu_ws/Startup'
    script_full = f'{startup_dir}/start.sh'

    start_container_cmd = ExecuteProcess(
        cmd=['docker', 'start', container_name],
        shell=True,
        output='screen'
    )

    # Define TERM para evitar mensagens "open terminal failed"
    tmux_cmd = (
        f'export TERM=xterm-256color; '
        f'tmux new-session -d -s simauto "cd {startup_dir} && bash {script_full}" '
        f'|| (tmux kill-session -t simauto; '
        f'export TERM=xterm-256color; '
        f'tmux new-session -d -s simauto "cd {startup_dir} && bash {script_full}")'
    )

    start_simulation_cmd = ExecuteProcess(
        cmd=['docker', 'exec', container_name, 'bash', '-lc', tmux_cmd],
        shell=True,
        output='screen'
    )

    navigator_node = Node(
        package='drone_navigation',
        executable='navigator',
        name='navigator_node',
        output='screen'
    )

    stop_container_cmd = ExecuteProcess(
        cmd=['docker', 'stop', container_name],
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg=f"Iniciando container: {container_name}"),
        start_container_cmd,

        ExecuteProcess(cmd=['sleep', '4'], shell=True),

        LogInfo(msg=f"Iniciando sessão tmux detached e rodando: {script_full}"),
        start_simulation_cmd,

        # Ajuste se a simulação demorar mais para subir
        ExecuteProcess(cmd=['sleep', '25'], shell=True),

        LogInfo(msg="Iniciando nó de navegação..."),
        navigator_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=navigator_node,
                on_exit=[
                    LogInfo(msg=f"Navegação terminou. Parando container {container_name}..."),
                    stop_container_cmd
                ]
            )
        )
    ])