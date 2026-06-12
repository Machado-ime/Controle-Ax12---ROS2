Esses cótigos são utilitários ros úteis para identificar problemas e estudar programas

# Códigos úteis

ros2 node list                  ---> Listar nós ativos
ros2 node info /nome_do_no      ---> informações de Publishers,Subscribers,Service Server,serviços ele chama
ros2 topic list                 ---> Listar tópicos ativos
ros2 topic info /nome_do_topico ---> informações de Publishers,Subscribers,QoS
ros2 topic echo /nome_do_topico ---> qual mensagem o tópico recebe (Publishers)
ros2 topic type /nome_do_topico ---> Ver o tipo de mensagem de um tópicos

# Dicionario

Publishers       ---> de quem o nó análisado recebe
Subscribers      ---> para quem o nó análisado manda
Service Server   ---> Nó que oferece o serviço
Service Client   ---> Nó que chama o serviço
QoS              ---> Qualidade de Serviço

# Ler info

Type:                ---> tipo da mensagem que que o tópico trabalha
Publisher count:     ---> número de nós ativos quem enviam mensagem para o tópico analisado
Subscription count:  ---> número de nós ativos quem recebem mensagem para o tópico analisado
QoS profile:
  Reliability:       ---> Define se o DDS garante a entrega da mensagem.
  Durability:        ---> Define se mensagens antigas são entregues a novos Subscribers.
  History:           ---> Define quantas mensagens ficam armazenadas
  Depth: 10

\Reliability

Reliable        ---> O sistema garante que a mensagem chega, Se perder, ele reenvia
Best Effort     ---> Envia e pronto, Se perder, perdeu

\Durability

Volatile        ---> Só recebe mensagens a partir do momento que se conecta, Mensagens antigas são descartadas
Transient Local ---> O publisher guarda a última(s) mensagem(ns), Um novo subscriber recebe imediatamente

\History

Keep Last       ---> Guarda apenas as N últimas mensagens, O valor de N vem do Depth
Keep All        ---> Guarda todas as mensagens