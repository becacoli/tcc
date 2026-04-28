# Planejamento de Caminho com Algoritmos Baseados em Amostragem

Este repositório contém a implementação e a avaliação de algoritmos de planejamento de caminho baseados em amostragem para o TCC, incluindo testes em ambientes 2D e integração com o CoppeliaSim para navegação simulada de um robô móvel `PioneerP3DX`.

O objetivo do projeto é comparar diferentes estratégias de planejamento em cenários com obstáculos, corredores e passagens estreitas, avaliando não apenas se o algoritmo encontra um caminho, mas também a qualidade, a segurança geométrica e a viabilidade de execução do caminho no simulador.

---

## Visão Geral

O projeto é focado em planejamento de caminho em ambientes 2D com obstáculos usando algoritmos baseados em amostragem. O planejador principal é implementado em Python, enquanto o CoppeliaSim é usado como ambiente de simulação para validar a execução dos caminhos planejados por um robô móvel.

Atualmente, o repositório inclui:

- `RRT`
- `RRT*`
- `RRT-Connect`
- `Informed RRT*`
- `RRT*-Smart`
- `EST`
- `EST-Híbrido`

O trabalho também inclui:

- experimentos em lote com múltiplas sementes;
- comparação entre cenários com diferentes configurações geométricas;
- navegação no CoppeliaSim usando a geometria da cena como obstáculo;
- visualização da árvore de busca e do caminho planejado;
- exportação e análise de métricas para uso no relatório do TCC.

---

## Cenas Utilizadas no CoppeliaSim

O projeto utiliza mais de uma cena de simulação no CoppeliaSim para avaliar os algoritmos em configurações geométricas diferentes.

### Cenário 1: corredor com parede angular

A primeira cena é um ambiente semelhante a um corredor, contendo:

- um robô móvel `PioneerP3DX`;
- seis paredes: `wall_1` até `wall_6`;
- um alvo representado pelo objeto `/GoalConfiguration`;
- um piso usado para definir os limites do mundo.

A figura abaixo mostra esse cenário:

![Cena do CoppeliaSim usada no projeto](path-planning/images/scene.png)

Nessa configuração:

- o robô inicia em uma região do ambiente;
- o objetivo é definido pela posição de `/GoalConfiguration`;
- as paredes formam passagens estreitas e estruturas semelhantes a becos sem saída;
- o planejador lê a geometria das paredes da cena e a converte em obstáculos 2D para o planejamento.

### Cenário 2: corredor com barreiras retangulares e obstáculos arredondados

O segundo cenário adiciona maior variedade geométrica ao ambiente:

- paredes externas `wall_1 ... wall_8`;
- barreiras retangulares internas;
- obstáculos arredondados posicionados na região central;
- robô e objetivo em regiões distintas do ambiente.

Esse cenário é útil para avaliar como os algoritmos se comportam com obstáculos de formatos diferentes e com múltiplas rotas possíveis.

![Segundo cenário do CoppeliaSim](path-planning/images/scene2.png)

### Cenário 3: ambiente aberto com obstáculo interno em ângulo

O terceiro cenário é mais simples geometricamente, mas útil para avaliar desvio e escolha de lado em torno de um obstáculo:

- paredes externas delimitando a área de navegação;
- um obstáculo interno em formato angular;
- espaço livre mais amplo ao redor do obstáculo;
- maior liberdade para comparar o formato dos caminhos gerados.

Esse cenário ajuda a analisar eficiência de caminho, suavidade e tendência de exploração.

![Terceiro cenário do CoppeliaSim](path-planning/images/scene3.png)

---

## O Que Já Foi Implementado

### Algoritmos

- `RRT`: planejador básico com uma única árvore e amostragem aleatória uniforme com viés para o objetivo.
- `RRT*`: extensão do RRT com escolha de melhor pai e etapa de `rewiring`, buscando reduzir o custo do caminho.
- `RRT-Connect`: planejador bidirecional que expande duas árvores, uma a partir do início e outra a partir do objetivo.
- `Informed RRT*`: restringe a região de amostragem após a primeira solução, usando uma elipse informada entre início e objetivo.
- `RRT*-Smart`: utiliza refinamento guiado por `beacons` ao redor do melhor caminho encontrado.
- `EST`: implementação do algoritmo `Expansive Space Trees`, que favorece a expansão de regiões menos densamente exploradas.
- `EST-Híbrido`: variante experimental deste trabalho, combinando expansão local baseada em densidade com uma taxa controlada de amostragem global.

### Infraestrutura de Avaliação

- cenários 2D pré-definidos para testes controlados;
- execução em lote com múltiplas sementes;
- visualização da árvore de busca e do caminho final;
- métricas de comprimento de caminho, suavidade, folga, tempo de planejamento e eficiência;
- exportação de resultados em formatos adequados para análise posterior.

### Integração com o CoppeliaSim

- conexão com o CoppeliaSim via ZMQ Remote API;
- leitura da posição do robô `PioneerP3DX`;
- leitura da posição do objetivo a partir de `/GoalConfiguration`;
- leitura da geometria das paredes da cena ativa;
- planejamento com a implementação em Python do algoritmo selecionado;
- seguimento dos waypoints gerados pelo planejador;
- métricas de execução, como erro final, tempo de execução, necessidade de replanejamento e motivo de término.

---

## EST e EST-Híbrido

O `EST` (`Expansive Space Trees`) é um algoritmo de planejamento baseado em árvore. Diferente do `RRT`, que expande a árvore em direção a amostras globais aleatórias, o EST seleciona regiões da árvore levando em conta a densidade local de nós. A ideia é favorecer a expansão de regiões menos exploradas do espaço livre.

A versão `EST-Híbrido` foi adicionada como uma variante experimental deste trabalho. Ela combina duas estratégias:

1. **Expansão local baseada em densidade**, seguindo a ideia do EST.
2. **Amostragem global probabilística**, com taxa controlada pelo parâmetro `global_sample_rate`.

Em termos simples, a cada iteração:

```text
Com probabilidade α:
    executa uma expansão global semelhante à do RRT.

Com probabilidade 1 - α:
    executa uma expansão local baseada em densidade, semelhante ao EST.
```

No código, o parâmetro `α` corresponde a:

```text
--global-sample-rate
```

Essa versão híbrida é útil para o TCC por dois motivos:

- do ponto de vista prático, permite equilibrar exploração global e expansão local;
- do ponto de vista teórico, facilita a discussão de completude probabilística, pois mantém uma componente de amostragem global com probabilidade positiva.

Importante: o termo `EST-Híbrido` é utilizado neste projeto para se referir à variante experimental implementada no repositório. Ele não deve ser apresentado como um algoritmo canônico da literatura, mas como uma adaptação proposta para avaliação comparativa.

---

## Como Está a Amostragem em Cada Algoritmo

As implementações atuais usam as seguintes estratégias de amostragem:

### `RRT`

Usa amostragem uniforme no mapa com viés para o objetivo. Com probabilidade `goal_sample_rate`, a amostra é o próprio objetivo; caso contrário, o ponto é sorteado uniformemente dentro dos limites do mapa. O número máximo de amostras é controlado por `max_iter`.

### `RRT*`

Usa a mesma amostragem do `RRT`: uniforme no mapa com viés para o objetivo. A diferença principal está na escolha do melhor pai para cada novo nó e na etapa de `rewiring`, que tenta reduzir o custo acumulado do caminho.

### `RRT-Connect`

Usa amostragem uniforme com viés para o objetivo, mas trabalha com duas árvores: uma a partir da configuração inicial e outra a partir do objetivo. Uma árvore é expandida em direção à amostra e a outra tenta se conectar agressivamente ao novo nó.

### `Informed RRT*`

Antes da primeira solução, usa amostragem uniforme com viés para o objetivo. Depois que encontra um caminho inicial, passa a amostrar dentro de uma elipse informada entre início e objetivo, concentrando a busca em regiões mais promissoras.

### `RRT*-Smart`

Usa a mesma amostragem do `RRT*` no início. Depois que encontra um bom caminho, cria `beacons` internos ao longo da solução atual e passa a amostrar perto desses pontos com probabilidade `beacon_sample_rate`.

### `EST`

Seleciona nós da árvore favorecendo regiões menos densas. A partir do nó selecionado, gera uma amostra local dentro de uma vizinhança definida por `local_sample_radius`. A densidade é avaliada usando `density_radius`.

### `EST-Híbrido`

Combina duas formas de expansão:

- expansão local do EST, baseada em densidade;
- expansão global, semelhante à do RRT, executada com probabilidade `global_sample_rate`.

Essa combinação busca reduzir a dependência exclusiva da amostragem local e aumentar a capacidade de exploração do espaço livre.

Em todas as variantes, a amostra sorteada não vira nó diretamente. O algoritmo usa a função `steer` para avançar apenas um passo de tamanho `step_size` na direção da amostra, e só adiciona o novo nó se o segmento gerado for livre de colisão.

---

## Instalação

Configuração básica no Windows/PowerShell:

```powershell
cd path-planning
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

---

## Uso

### Experimentos em lote em 2D

Exemplo com `RRT*`:

```powershell
python -m experiments.run_rrt_batch --algo rrt_star --scenario corredor --runs 20
```

Exemplos com outros algoritmos:

```powershell
python -m experiments.run_rrt_batch --algo rrt --scenario complexo --runs 20
python -m experiments.run_rrt_batch --algo rrt_connect --scenario complexo --runs 20
python -m experiments.run_rrt_batch --algo informed_rrt_star --scenario complexo --runs 20
python -m experiments.run_rrt_batch --algo rrt_star_smart --scenario corredor --runs 20 --beacon-sample-rate 0.35 --beacon-radius 8
python -m experiments.run_rrt_batch --algo est --scenario passagem_estreita --runs 20
python -m experiments.run_rrt_batch --algo est_hybrid --scenario passagem_estreita --runs 20 --global-sample-rate 0.35
```

### Teste de conexão com o CoppeliaSim

Com a cena carregada e a simulação em execução:

```powershell
python coppeliasim/connect_test.py
```

Também é possível listar os objetos da cena ativa:

```powershell
python discover_scene_objects.py
```

### Execução no CoppeliaSim com `RRT-Connect`

```powershell
python coppeliasim/rrt_navigation.py `
  --algo rrt_connect `
  --robot-path /PioneerP3DX `
  --left-motor-path /PioneerP3DX/leftMotor `
  --right-motor-path /PioneerP3DX/rightMotor `
  --goal-object /GoalConfiguration `
  --obstacle-model polygon `
  --polygon-source vertices `
  --robot-radius 0.22 `
  --wall-inflate 0.00 `
  --step-size 1.2 `
  --goal-sample-rate 0.15 `
  --max-iter 8000 `
  --plan-attempts 20 `
  --waypoint-spacing 0.06 `
  --lookahead-waypoints 1 `
  --linear-speed 0.12 `
  --debug `
  --plot-planner
```

### Execução no CoppeliaSim com `RRT*`

```powershell
python coppeliasim/rrt_navigation.py `
  --algo rrt_star `
  --robot-path /PioneerP3DX `
  --left-motor-path /PioneerP3DX/leftMotor `
  --right-motor-path /PioneerP3DX/rightMotor `
  --goal-object /GoalConfiguration `
  --obstacle-model polygon `
  --polygon-source vertices `
  --robot-radius 0.20 `
  --wall-inflate 0.01 `
  --step-size 0.9 `
  --neighbor-radius 8 `
  --goal-sample-rate 0.15 `
  --max-iter 8000 `
  --plan-attempts 15 `
  --waypoint-spacing 0.04 `
  --lookahead-waypoints 1 `
  --linear-speed 0.08 `
  --control-collision-check `
  --slowdown-clearance 0.12 `
  --debug `
  --plot-planner
```

### Execução no CoppeliaSim com `EST-Híbrido`

```powershell
python coppeliasim/rrt_navigation.py `
  --algo est_hybrid `
  --robot-path /PioneerP3DX `
  --left-motor-path /PioneerP3DX/leftMotor `
  --right-motor-path /PioneerP3DX/rightMotor `
  --goal-object /GoalConfiguration `
  --obstacle-model polygon `
  --polygon-source vertices `
  --robot-radius 0.18 `
  --wall-inflate 0.00 `
  --step-size 1.0 `
  --density-radius 4 `
  --local-sample-radius 3 `
  --global-sample-rate 0.45 `
  --density-candidates 40 `
  --goal-sample-rate 0.15 `
  --max-iter 15000 `
  --plan-attempts 25 `
  --waypoint-spacing 0.04 `
  --lookahead-waypoints 1 `
  --linear-speed 0.08 `
  --control-collision-check `
  --control-clearance 0.03 `
  --tracking-clearance 0.04 `
  --slowdown-clearance 0.12 `
  --debug `
  --plot-planner
```

---

## Métricas Utilizadas

O projeto utiliza métricas para avaliar planejamento, execução e segurança geométrica.

### Métricas principais

- `success`  
  Indica se a execução foi considerada bem-sucedida. Deve ser `True` apenas quando o robô alcança o objetivo dentro de `goal_tolerance` e a execução não termina por travamento, falha de replanejamento ou parada por risco de colisão.

- `planning_time`  
  Tempo total de planejamento em segundos.

- `execution_time`  
  Tempo total gasto para o robô executar o caminho no CoppeliaSim.

- `path_length`  
  Comprimento total do caminho, calculado como a soma das distâncias euclidianas entre pontos consecutivos.

- `path_smoothness`  
  Mede o quanto o caminho muda de direção. É calculada pela soma das variações angulares entre segmentos consecutivos. Quanto menor, mais suave é o caminho.

- `num_nodes`  
  Número de nós gerados pela árvore durante o planejamento.

- `replans`  
  Número de vezes em que o robô precisou replanejar durante a execução.

- `final_goal_error`  
  Distância final entre o robô e o objetivo.

- `path_efficiency`  
  Mede o quão direto é o caminho, geralmente pela razão entre distância em linha reta e comprimento total do caminho. Quanto mais próximo de 1, mais direto é o trajeto.

- `termination_reason`  
  Motivo de encerramento da execução. Ajuda a distinguir sucesso, falha de planejamento, travamento, parada por risco de colisão ou falha de replanejamento.

### Métricas de folga e segurança

As métricas de folga são importantes porque um caminho pode ser geometricamente válido no planner, mas ainda assim ser difícil de executar no CoppeliaSim se passar muito próximo das paredes.

- `clearance` ou `min_clearance`  
  Menor distância entre o caminho planejado e os obstáculos. Quanto maior esse valor, maior a folga geométrica do caminho.

- `mean_clearance`  
  Distância média do caminho até os obstáculos. Ajuda a diferenciar caminhos que possuem apenas um ponto crítico daqueles que permanecem próximos das paredes durante grande parte do trajeto.

- `clearance_std`  
  Desvio padrão da folga ao longo do caminho. Pode indicar instabilidade na distância em relação aos obstáculos.

- `min_tracking_clearance`  
  Menor distância entre a trajetória realmente executada pelo robô no CoppeliaSim e os obstáculos. Essa métrica é mais prática que o clearance do planner, pois considera erro de rastreamento e dinâmica do robô.

- `collision_or_contact`  
  Indica se houve colisão, contato ou raspagem perceptível durante a execução.

- `near_obstacle_time`  
  Tempo ou percentual da execução em que o robô permaneceu abaixo de um limite mínimo de folga, por exemplo `0.03 m` ou `0.05 m`.

- `safety_margin_ratio`  
  Razão entre a folga mínima e o raio do robô. Pode ser usada para comparar cenários e algoritmos com diferentes configurações.

Essas métricas ajudam a comparar algoritmos além da taxa de sucesso. Por exemplo, dois algoritmos podem alcançar o objetivo, mas um pode gerar caminhos mais curtos e próximos das paredes, enquanto outro pode gerar caminhos mais longos e seguros.

---

## Próximos Passos do TCC

Os próximos passos do trabalho são:

- [ ] Executar experimentos comparativos completos com múltiplas sementes.
- [ ] Comparar os algoritmos nos três cenários do CoppeliaSim.
- [ ] Consolidar tabelas e gráficos com as métricas de planejamento e execução.
- [ ] Incluir métricas de folga, como `min_clearance`, `mean_clearance` e `min_tracking_clearance`.
- [ ] Analisar casos em que o planner encontra caminho, mas o robô raspa ou passa muito próximo das paredes.
- [ ] Formalizar o problema matemático de planejamento em espaço de configuração.
- [ ] Apresentar a noção de completude probabilística.
- [ ] Provar ou discutir a completude probabilística dos algoritmos implementados.
- [ ] Dar foco especial à análise do `RRT*` e do `EST-Híbrido`.
- [ ] Discutir as limitações práticas observadas no CoppeliaSim.

---

## Prova de Completude Probabilística

A próxima etapa teórica do TCC será relacionar a implementação com a noção de completude probabilística dos algoritmos baseados em amostragem.

A ideia geral é demonstrar que, se existe um caminho viável com folga positiva, a probabilidade do algoritmo encontrar uma solução tende a 1 quando o número de amostras tende ao infinito.

Para isso, o trabalho deverá explicitar hipóteses como:

- o espaço de configuração é limitado;
- o espaço livre possui medida positiva;
- existe uma solução com `clearance` positivo;
- o amostrador possui suporte em todo o espaço livre;
- o planejador local valida corretamente os segmentos entre configurações;
- o verificador de colisão é consistente;
- a região objetivo possui tolerância positiva.

### Foco no `RRT*`

O `RRT*` será analisado quanto à completude probabilística e discutido em relação à sua propriedade de melhoria de custo por meio do `rewiring`.

A análise deve deixar claro que:

- o `RRT*` mantém a amostragem global do RRT;
- a etapa de `rewiring` melhora o custo, mas não impede a exploração do espaço livre;
- a completude probabilística pode ser discutida sob as hipóteses clássicas de amostragem e existência de solução robusta;
- a otimalidade assintótica depende de condições adicionais, especialmente sobre o raio de vizinhança.

### Foco no `EST-Híbrido`

O `EST-Híbrido` será analisado como variante proposta neste trabalho.

A completude probabilística será discutida a partir da presença de uma componente global de amostragem com probabilidade positiva `α`, definida por:

```text
α = global_sample_rate
```

Como `α > 0`, existe sempre uma chance fixa de o algoritmo executar uma expansão global semelhante à do RRT. Assim, mesmo que a estratégia local baseada em densidade falhe em algumas regiões, a componente global preserva a possibilidade de explorar qualquer região livre de volume positivo.

Em outras palavras, o argumento central é:

```text
Se existe uma solução robusta com folga positiva
e o EST-Híbrido mantém α > 0,
então a probabilidade de alcançar a região objetivo tende a 1
quando o número de amostras tende ao infinito.
```

---

## Roadmap

- [x] Implementar `RRT`
- [x] Implementar `RRT*`
- [x] Implementar `RRT-Connect`
- [x] Implementar `Informed RRT*`
- [x] Implementar `RRT*-Smart`
- [x] Implementar `EST`
- [x] Implementar `EST-Híbrido`
- [x] Integrar os planejadores com o CoppeliaSim
- [x] Adicionar métricas de experimento e exportação de resultados
- [ ] Executar experimentos comparativos completos com múltiplas sementes
- [ ] Adicionar e consolidar métricas de folga e segurança
- [ ] Consolidar tabelas e gráficos para o relatório do TCC
- [ ] Formalizar as hipóteses matemáticas dos planejadores
- [ ] Provar/discutir completude probabilística dos algoritmos
- [ ] Discutir limitações práticas entre planejamento geométrico e execução no CoppeliaSim

---

## Observação

O planejador usado nos experimentos é a implementação em Python deste repositório. O CoppeliaSim é usado como ambiente de simulação e como fonte de geometria da cena, e não como algoritmo principal de planejamento.

As provas matemáticas devem ser entendidas como propriedades do modelo abstrato de planejamento e da implementação geométrica do planner. A execução no CoppeliaSim serve como validação experimental, podendo revelar limitações adicionais relacionadas ao controle, à dinâmica do robô e ao rastreamento dos waypoints.
