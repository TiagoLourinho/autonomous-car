# Robotics-Autonomous-Car Main Components

## Goal
This lab assignment aims at developing components for an autonomous car operating inside IST Alameda campus. 

## Authors
- Francisco Rodrigues
- João Marafuz Gaspar
- João Fonseca
- Manuel Graça
- Tiago `Ambrósio` Lourinho (Senior Engineer)
- Tiago Ferreira

The role of the rest of the group is considered as `slave`.

## Bloco planeamento trajetória

### Inputs: 
- Mapa
- Posição atual
- Posição destino

### Output:
- Trajetória (conjunto de posições)

### Notas:
- Usar API do Google maps

## Bloco controlo

### Input:
- Próximo ponto da trajetória
- Estimativa de posição e velocidade atual

### Output: 
- Angulo do volante

### Notas: 
- Assume se velocidade constante
- O volante pode dar várias voltas, dps vai ter de ser medido experimentalmente a ligação com o ângulo de output do bloco de controlo 

## Modelo do carro

### Notas:
- Usar o modelo simples dos slides (podia ser modelo dinâmico mas 4 semanas...)
- Há um modelo bom CARLA, mas é difícil de implementar e não sei se vale a pena...

## Bloco estimativa posição

### Inputs: 
- Info dos sensores
- Estimativa do modelo

### Output:
- Estimativa final de posição/velocidade

### Notas:
- Usar kalman filter/ filtro de partículas


## Sensores

### Notas:
- IMU (relativo) e GPS (absoluto)
- Temos de comprar IMU, e dps comunica com o PC através do arduino com IC2
- GPS devíamos tentar usar o do telemóvel para não estarmos dependentes do do stor
- Comunicao e com porta série com o carro/GPS do stor
- Tem de ser tido em conta o timing/sincronização da informação, não apenas nos sensores mas como em tudo ao resto para as diferentes partes baterem certo