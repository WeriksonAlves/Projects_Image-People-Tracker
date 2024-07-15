# Controle Servo-Visual Onboard para Reconhecimento de Ações

Este repositório contém o projeto desenvolvido por Wérikson Frederiko de Oliveira Alves na disciplina ELT791 - Tópicos Especiais - Robótica e suas Utilidades na Universidade Federal de Viçosa. O projeto foca no controle servo-visual acoplado a um drone para o reconhecimento de ações humanas.

## Sumário

- [Introdução](#introdução)
- [Objetivos](#objetivos)
- [Metodologia](#metodologia)
  - [Suporte da Câmera](#suporte-da-câmera)
  - [Sistema de Reconhecimento de Gestos](#sistema-de-reconhecimento-de-gestos)
  - [Configuração da ESP32CAM](#configuração-da-esp32cam)
  - [Sistema de Reconhecimento com Orientação da Câmera](#sistema-de-reconhecimento-com-orientação-da-câmera)
- [Resultados Alcançados](#resultados-alcançados)
- [Considerações Finais](#considerações-finais)
- [Contato](#contato)

## Introdução

Neste projeto, enfrentamos os desafios da interação humano-robô, especialmente no contexto de controle por gestos usando drones. O objetivo principal é manter o operador sempre no campo de visão do drone sem restringir os graus de liberdade, utilizando um sistema de reconhecimento de ações e um controle servo-visual para a câmera.

## Objetivos

- **Manter o operador sempre no campo de visão do drone.**
- **Criar um módulo externo para fixar a câmera durante missões.**
- **Atualizar o sistema de reconhecimento de gestos para captura de imagens de fontes alternativas.**
- **Implementar sistema de controle servo-visual para a câmera.**

## Metodologia

### Suporte da Câmera

Implementação de um suporte para fixação da câmera durante as missões de voo.

### Sistema de Reconhecimento de Gestos

O sistema utiliza o modelo YOLOv8 para detecção de objetos e rastreamento do operador, MediaPipe Hands para rastreamento da mão, e BlazePose para rastreamento do corpo. A classificação de gestos é feita utilizando um classificador kNN.

### Configuração da ESP32CAM

Configuração da ESP32CAM para transmissão de imagens via streaming através da rede Wi-Fi. Implementação de configuração OTA para atualização remota do código e controle do servo motor usando ROS.

### Sistema de Reconhecimento com Orientação da Câmera

Atualização do sistema de reconhecimento para incluir novas classes que lidam com a comunicação do ROS, inicialização do servo, e leitura das imagens transmitidas. Implementação de uma rotina de controle proporcional para manter o operador no centro do campo de visão da câmera.

## Resultados Alcançados

- **Aquisição de Imagem:** Identificação de limitações como instabilidade, lentidão e distância do ponto de acesso.
- **ESP32CAM:** Avaliação das limitações de hardware, especialmente em relação à qualidade e frequência de transmissão.
- **Módulo de Suporte:** Eficácia e desafios observados durante os experimentos.
- **Controle do Servo Motor:** Desempenho satisfatório com áreas identificadas para melhorias futuras.

## Considerações Finais

O sistema desenvolvido é funcional como protótipo, mas requer aprimoramentos em áreas como aquisição de imagens e limitações de hardware do ESP32CAM. O suporte e controle do servo motor mostraram um bom desempenho, indicando potencial para futuras melhorias.

## Contato

Wérikson F. de O. Alves  
Universidade Federal de Viçosa  
Núcleo de Especialização em Robótica (NERo)  
Email: werikson.alves@ufv.br  
LinkedIn: [werikson-alves](https://www.linkedin.com/in/werikson-alves)  
Twitter: [@roboticaUFV](https://twitter.com/roboticaUFV)

---

