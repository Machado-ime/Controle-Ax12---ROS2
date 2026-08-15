cmake_minimum_required(VERSION 3.5)
project(adam_bringup)

find_package(ament_cmake REQUIRED)

# Instalar todos os diretórios do pacote
# (obrigatório no ROS 2 — não existe devel space automático)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
