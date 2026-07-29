#!/usr/bin/env python3
"""Roda uma marcha em ciclo continuo no robo REAL, com largada controlada:
ao iniciar, o robo vai para a coluna 1 (posicao inicial) e ESPERA ali ate
o usuario dar play — só depois disso entra no ciclo continuo (coluna 2, 3,
..., volta pra 1, repete) igual ao send_gait.py.

Reaproveita ConexaoRobo/carregar_marcha/carregar_pausas/carregar_passos/
calcular_velocidade_rad_s do send_gait.py — mesma conexao, mesmo formato
de matriz, mesma logica de velocidade proporcional (juntas chegam
juntas). Duracao e pausa por coluna: se o YAML tiver as chaves opcionais
'passos'/'pausas' (listas, 1 valor por coluna), cada coluna usa
passos[i]+pausas[i] em vez dos escalares 'passo'/'pausa' fixos — util
para etapas de voo mais longas que as de peso, por exemplo.

Uso (com o ax12_controller ja rodando em outro terminal/launch):
    ros2 run ax12_control marcha_continua --ros-args -p matriz:=marcha_pe
    ros2 run ax12_control marcha_continua --ros-args -p matriz:=marcha_pe -p velocidade_inicial:=0.2
"""

import threading

import rclpy
import yaml

from ax12_control.send_gait import (
    ConexaoRobo,
    calcular_velocidade_rad_s,
    carregar_marcha,
    carregar_pausas,
    carregar_passos,
)


def main():
    robo = ConexaoRobo()
    robo._node.declare_parameter('velocidade_inicial', 0.3)  # rad/s, so p/ ir a coluna 1
    velocidade_inicial = robo._node.get_parameter('velocidade_inicial').value

    try:
        nomes_juntas, matriz_movimento, passo, pausa = carregar_marcha(
            robo.arquivo_marcha)
    except FileNotFoundError:
        print(f'ERRO: arquivo de marcha nao encontrado: {robo.arquivo_marcha}')
        robo.fechar_conexao()
        return
    except (yaml.YAMLError, ValueError, TypeError) as e:
        print(f'ERRO no arquivo de marcha ({robo.arquivo_marcha}): {e}')
        robo.fechar_conexao()
        return

    print(f'Marcha carregada de: {robo.arquivo_marcha}')
    num_points = len(matriz_movimento[0])
    pausas = carregar_pausas(robo.arquivo_marcha, num_points, pausa)
    passos = carregar_passos(robo.arquivo_marcha, num_points, passo)

    try:
        robo.aguardar_controlador()

        # --- Vai para a posicao inicial (coluna 1) e para ali ---
        posicoes_anteriores = [linha[0] for linha in matriz_movimento]
        robo.enviar_passo(
            nomes_juntas, posicoes_anteriores,
            [velocidade_inicial] * len(nomes_juntas), passos[0])
        tempo_espera_inicial = passos[0] + pausas[0]
        print(f'Indo para a posicao inicial (coluna 1)... '
              f'aguardando {tempo_espera_inicial:.2f}s')
        robo.esperar(tempo_espera_inicial)

        if robo.falha_fatal:
            print('\nFalha fatal antes do play. Abortando.')
            return

        # --- Espera o play (ENTER), sem parar de processar a rede ---
        print('\nNa posicao inicial. Pressione ENTER para iniciar o ciclo '
              'continuo (Ctrl+C para sair sem iniciar).')
        play = threading.Event()

        def _aguardar_enter():
            try:
                input()
            except EOFError:
                pass
            play.set()

        threading.Thread(target=_aguardar_enter, daemon=True).start()

        while not play.is_set() and not robo.falha_fatal:
            rclpy.spin_once(robo._node, timeout_sec=0.1)

        if robo.falha_fatal:
            print('\nFalha fatal antes do play. Abortando.')
            return

        # --- Play: ciclo continuo, comeca pela coluna 2 (ja estamos na 1) ---
        print('Play! Iniciando ciclo continuo... (Ctrl+C para parar)')
        current_index = 1 % num_points

        while not robo.falha_fatal:
            posicoes_alvo = []
            velocidades_alvo = []

            passo_coluna = passos[current_index]
            for i in range(len(nomes_juntas)):
                rad_alvo = matriz_movimento[i][current_index]
                vel_rad = calcular_velocidade_rad_s(
                    rad_alvo, posicoes_anteriores[i], passo_coluna)
                posicoes_alvo.append(rad_alvo)
                velocidades_alvo.append(vel_rad)
                posicoes_anteriores[i] = rad_alvo

            robo.enviar_passo(nomes_juntas, posicoes_alvo, velocidades_alvo, passo_coluna)
            tempo_espera = passo_coluna + pausas[current_index]
            print(f'Passo {current_index + 1}/{num_points} enviado | '
                  f'Aguardando {tempo_espera:.2f}s')

            current_index = (current_index + 1) % num_points
            robo.esperar(tempo_espera)

        print('\nMarcha INTERROMPIDA: o controlador desistiu do hardware. '
              'Verifique o robo e reinicie os dois nos.')

    except KeyboardInterrupt:
        print('\nScript interrompido pelo usuário.')
    finally:
        robo.fechar_conexao()


if __name__ == '__main__':
    main()
