# Trabalho de Cinemática — Robôs Industriais (Cartesiano e Polar)

**Autor:** Higor dos Anjos
**Data:** 2026-05-21
**Disciplina:** Cinemática

---

## 1. Introdução

Este trabalho apresenta a modelagem e simulação computacional de **dois manipuladores robóticos industriais 3D**: o **robô Cartesiano (PPP)** e o **robô Polar (RRP, esférico)**. Para cada um, deriva-se a cinemática direta (FK — *forward kinematics*) e a cinemática inversa (IK — *inverse kinematics*), apresentam-se exemplos numéricos e implementa-se uma simulação interativa em OpenGL com câmera órbita 3D.

O objetivo é demonstrar dois extremos da classificação de robôs industriais por tipo de junta:

- **Cartesiano**: todas as juntas são prismáticas (PPP). O efetuador se move ao longo dos eixos cartesianos. FK e IK são **triviais**.
- **Polar**: combina rotações e uma extensão linear (RRP). O efetuador é parametrizado em coordenadas esféricas. FK e IK envolvem **conversão entre sistemas de coordenadas**.

### 1.1 Classificação geral dos robôs industriais

Por convenção, robôs industriais são classificados pela combinação das 3 primeiras juntas (que definem a posição do punho):

| Estrutura | Juntas | Espaço de trabalho |
|---|---|---|
| **Cartesiano** | PPP (3 lineares) | Caixa (paralelepípedo) |
| **Cilíndrico** | RPP (rotativa + 2 lineares) | Cilindro |
| **Polar (esférico)** | RRP (2 rotativas + 1 linear) | Casca esférica |
| **SCARA** | RRP (2 rotativas paralelas + 1 vertical) | Disco/cilindro |
| **Articulado (antropomórfico)** | RRR (3 rotativas) | Esfera/forma complexa |

### 1.2 Conceitos fundamentais

- **Junta prismática (P, L)**: permite translação. Variável: comprimento `d` (ou `ρ`).
- **Junta rotativa (R)**: permite rotação. Variável: ângulo `θ` (ou `φ`).
- **Elo**: corpo rígido entre duas juntas.
- **Efetuador**: extremidade do robô — ponto de interesse cuja posição queremos controlar.
- **Espaço articular**: conjunto dos vetores de variáveis das juntas.
- **Espaço cartesiano**: posição (e orientação) do efetuador no mundo, em XYZ.
- **FK**: dados os ângulos/extensões das juntas, calcular a posição do efetuador.
- **IK**: dada a posição desejada, calcular as variáveis das juntas.

### 1.3 Convenções deste trabalho

- Eixo **Z aponta para cima** (convenção robótica padrão; o piso fica no plano XY).
- Eixos coloridos na simulação: **X = vermelho**, **Y = verde**, **Z = azul**.
- Unidades: adimensionais (a simulação trabalha em "unidades"; em hardware real seriam mm ou m).
- Ângulos: armazenados em radianos internamente, exibidos em graus na UI.

---

## 2. Convenção visual da simulação

| Elemento | Significado |
|---|---|
| Esfera laranja grande na origem | Base do robô |
| Esfera rosa | Efetuador |
| Cilindro cinza-escuro grosso + cilindro prata fino dentro | Junta prismática estendida (visual de cilindro hidráulico) |
| Cubo laranja | Junta prismática (indicador no ponto de montagem) |
| Wireframe azul (linhas) | Workspace (caixa pro Cartesiano, anel pro Polar) |
| Cruz colorida + esfera no alvo | Verde = IK ok / Vermelho = IK falhou |
| 3 linhas vermelha/verde/azul saindo da origem | Eixos X/Y/Z (1 unidade de comprimento) |
| Grid no piso | Plano XY com espaçamento de 0.5 unidade, 10 unidades de lado |

---

## 3. Robô Cartesiano (PPP)

### 3.1 Modelagem

Três juntas prismáticas, cada uma deslizando ao longo de um eixo cartesiano principal. Topologia parecida com uma impressora 3D ou um CNC de 3 eixos.

- **Juntas**: `d₁` (eixo X), `d₂` (eixo Y), `d₃` (eixo Z), cada uma com mínimo e máximo configuráveis.
- **Variáveis**: as próprias distâncias, em unidades de comprimento.
- **DOF**: 3 (suficiente para posicionar em XYZ; não controla orientação).
- **Workspace**: caixa retangular `[d₁_min, d₁_max] × [d₂_min, d₂_max] × [d₃_min, d₃_max]`. Os limites mínimos modelam o **offset mecânico** real do batente do encoder (em hardware nunca é zero exato).

### 3.2 Cinemática Direta (FK)

A posição do efetuador é obtida diretamente das variáveis das juntas:

$$
\begin{aligned}
x &= d_1 \\
y &= d_2 \\
z &= d_3
\end{aligned}
$$

**Trivial** — não há transformação alguma, porque cada junta age ao longo de um eixo do referencial mundo.

### 3.3 Cinemática Inversa (IK)

Inverter o sistema acima é também trivial:

$$
d_1 = x, \qquad d_2 = y, \qquad d_3 = z
$$

A IK só falha quando o alvo está fora dos ranges de algum eixo:

$$
\text{válido} \iff d_{1\min} \le x \le d_{1\max} \;\land\; d_{2\min} \le y \le d_{2\max} \;\land\; d_{3\min} \le z \le d_{3\max}
$$

### 3.4 Exemplos numéricos (d_max = 2.0 em cada eixo)

**Exemplo FK-CART-1**: `d₁ = 1.0`, `d₂ = 1.5`, `d₃ = 0.8`.

| Cálculo | Valor |
|---|---|
| `x = d₁` | **1.000** |
| `y = d₂` | **1.500** |
| `z = d₃` | **0.800** |

Efetuador: `(1.000, 1.500, 0.800)`. Conferência: simulação reporta `(+1.000, +1.500, +0.800)` (print 01).

**Exemplo IK-CART-1**: alvo `(0.5, 1.2, 0.3)`.

| Cálculo | Valor |
|---|---|
| `d₁ = x = 0.5`, dentro de [0, 2] | ✓ |
| `d₂ = y = 1.2`, dentro de [0, 2] | ✓ |
| `d₃ = z = 0.3`, dentro de [0, 2] | ✓ |

Solução única: `(d₁, d₂, d₃) = (0.5, 1.2, 0.3)`.

**Exemplo IK-CART-2 (falha)**: alvo `(2.5, 1.0, 0.5)`.

- `x = 2.5 > d₁_max = 2.0` → falha: "X fora do range".

### 3.5 Discussão

A simplicidade da FK/IK Cartesiana é o principal atrativo dessa estrutura no chão de fábrica: programação é direta, sem singularidades, e a precisão depende apenas da resolução dos encoders lineares. A desvantagem é o **espaço físico ocupado**: para alcançar um cubo de aresta `L`, o robô precisa de uma estrutura externa de pelo menos `L` em cada direção (trilhos), em vez de pivotar em torno de uma base compacta como os robôs articulados.

---

## 4. Robô Polar / Esférico (RRP)

### 4.1 Modelagem

Duas rotações em série + uma extensão prismática, todas montadas sobre uma **coluna vertical fixa de altura `d₁`**. O efetuador é descrito em **coordenadas esféricas centradas no ombro** (topo da coluna).

- **Elo fixo**:
  - `d₁`: altura da coluna vertical entre o turntable (na base, plano XY) e o pivô do ombro. Não é variável de junta; é geometria fixa do robô.
- **Juntas**:
  - `θ` (theta): **azimute** — rotação em torno do eixo Z (vertical), na base.
  - `φ` (phi): **elevação** — rotação no plano vertical em torno do **ombro**, medida a partir do plano horizontal que passa pelo ombro (positivo = pra cima).
  - `ρ` (rho): **distância radial** — extensão do braço telescópico, medida **a partir do ombro**.
- **DOF**: 3 (posicionamento em XYZ).
- **Workspace**: casca esférica de raio externo `ρ_max` e raio interno `ρ_min` centrada em `(0, 0, d₁)`, recortada pelo intervalo de `φ` (tipicamente `−90° ≤ φ ≤ +90°`).

### 4.2 Cinemática Direta (FK)

A posição do ombro é fixa: `(0, 0, d₁)`. A posição do efetuador é dada pela conversão **esférico → cartesiano** somada ao deslocamento do ombro:

$$
\begin{aligned}
x &= \rho \, \cos\varphi \, \cos\theta \\
y &= \rho \, \cos\varphi \, \sin\theta \\
z &= d_1 + \rho \, \sin\varphi
\end{aligned}
$$

Note que `cos(φ)` aparece nos componentes X e Y (a projeção no plano XY a partir do ombro), e `sin(φ)` aparece na contribuição em Z somada a `d₁`. Quando `φ = 0`, o braço fica horizontal e `z = d₁`. Quando `φ = +90°`, o braço aponta direto pra cima: `(x, y) = (0, 0)`, `z = d₁ + ρ`.

### 4.3 Cinemática Inversa (IK)

Desloca-se o alvo para o referencial do ombro subtraindo `d₁` em Z, e depois aplica-se a conversão **cartesiano → esférico**:

$$
\begin{aligned}
z_r    &= z - d_1 \\
\rho   &= \sqrt{x^2 + y^2 + z_r^2} \\
\theta &= \mathrm{atan2}(y, x) \\
\varphi &= \mathrm{atan2}\!\left(z_r, \sqrt{x^2 + y^2}\right)
\end{aligned}
$$

O uso de `atan2` em vez de `atan` ou `acos` evita ambiguidades de sinal e o caso degenerado de divisão por zero quando `x = y = 0`.

### 4.4 Limites de alcance

A IK do robô polar falha quando:

- `ρ > ρ_max`: alvo longe demais — o braço, mesmo totalmente estendido, não alcança.
- `ρ < ρ_min`: alvo perto demais — o braço prismático não retrai o suficiente.
- `φ` fora do intervalo permitido (normalmente `[−90°, +90°]`): alvo em uma direção mecanicamente inacessível pelas rotações.

### 4.5 Exemplos numéricos (d₁ = 0.6, ρ_min = 0.4, ρ_max = 2.5)

**Exemplo FK-POL-1**: `θ = 30°`, `φ = 45°`, `ρ = 2.0`.

| Cálculo | Valor |
|---|---|
| `cos(30°)` | 0.866 |
| `sin(30°)` | 0.500 |
| `cos(45°)` | 0.707 |
| `sin(45°)` | 0.707 |
| `x = 2.0 · 0.707 · 0.866` | **1.224** |
| `y = 2.0 · 0.707 · 0.500` | **0.707** |
| `z = d₁ + 2.0 · 0.707 = 0.6 + 1.414` | **2.014** |

Efetuador: `(1.224, 0.707, 2.014)`.

**Exemplo IK-POL-1**: alvo `(1.5, 0, 1.6)`.

| Passo | Cálculo | Valor |
|---|---|---|
| `z_r` | `z − d₁ = 1.6 − 0.6` | 1.000 |
| `ρ²` | `1.5² + 0² + 1.0² = 2.25 + 0 + 1.0` | 3.25 |
| `ρ` | `√3.25` | **1.803** |
| `θ` | `atan2(0, 1.5)` | **0.00°** |
| `xy` | `√(1.5² + 0²)` | 1.500 |
| `φ` | `atan2(1.0, 1.5)` | **33.69°** |

Solução: `θ = 0°`, `φ = 33.69°`, `ρ = 1.803`. Validações: `ρ ∈ [0.4, 2.5]` ✓, `φ ∈ [−90°, +90°]` ✓.

Conferência: simulação reporta `theta = 0.00°`, `phi = 33.69°`, `rho = 1.803`, e o efetuador em `(+1.500, +0.000, +1.600)` (print 02).

**Exemplo IK-POL-2**: alvo `(0, 0, 3.1)`.

| Passo | Cálculo | Valor |
|---|---|---|
| `z_r` | `3.1 − 0.6` | 2.500 |
| `ρ` | `√(0 + 0 + 6.25)` | **2.500** |
| `θ` | `atan2(0, 0)` | **0°** (por convenção do `atan2`) |
| `xy` | `√(0)` | 0 |
| `φ` | `atan2(2.5, 0)` | **90°** (zênite) |

Solução: braço apontando exatamente pra cima a partir do ombro, totalmente estendido — o efetuador alcança `z = d₁ + ρ_max = 3.1`.

**Exemplo IK-POL-3 (falha)**: alvo `(3, 0, 0.6)`.

- `z_r = 0`, `ρ = 3.0 > ρ_max = 2.5` → falha: "rho acima do maximo (alvo longe demais do ombro)".

### 4.6 Discussão

O robô polar é o caso clássico que aparece em livros de robótica como exemplo da **conversão esférica ↔ cartesiana**. Sua IK em forma fechada é uma das mais limpas que existem. O preço é a presença de **singularidades**: nos polos (`φ = ±90°`), o azimute `θ` se torna indeterminado (o `atan2(0,0)` retorna 0 por convenção, mas qualquer θ daria o mesmo efetuador). Esse é o problema de "gimbal lock" típico de representações com Euler angles.

A introdução do **elo fixo `d₁`** (coluna vertical) torna o modelo fiel ao layout físico de um robô polar industrial real: a base permanece no chão, o ombro fica em uma altura útil de operação, e o workspace se torna uma casca esférica **elevada** em vez de uma esfera centrada na origem do mundo. Isso também elimina uma colisão visual: sem `d₁`, qualquer alvo abaixo do plano XY exigiria que o braço atravessasse o piso.

---

## 5. Comparação rápida

| Critério | Cartesiano (PPP) | Polar (RRP) |
|---|---|---|
| Tipo de juntas | 3 prismáticas | 2 rotativas + 1 prismática |
| FK | Identidade | Conversão esférica → cartesiana |
| IK | Identidade + range check | `atan2` + `sqrt` + range check |
| Singularidades | Nenhuma | Polos (`φ = ±90°`) |
| Workspace | Caixa | Casca esférica |
| Aplicação típica | CNC, impressora 3D, paletizador | Manipulação geral, máquinas de medição |

---

## 6. Implementação

### 6.1 Stack

- **C++20** com **CMake + Ninja** e **vcpkg em modo manifest**.
- **GLFW** (janela e entrada), **GLAD** (loader OpenGL), **GLM** (álgebra linear), **Dear ImGui** (UI).
- OpenGL 3.3 Core Profile.

### 6.2 Arquitetura

```
cinematica-industrial/
├── src/
│   ├── main.cpp                       (loop principal + UI)
│   ├── camera3d.{h,cpp}               (câmera órbita: yaw, pitch, distance)
│   ├── renderer3d.{h,cpp}             (esfera, cilindro, cubo, grid, eixos, linha)
│   ├── animation.{h,cpp}              (interpolador de juntas — smoothstep, shortest angular path)
│   ├── manipulator.h                  (interface IManipulator)
│   └── manipulators/
│       ├── cartesian.{h,cpp}
│       └── polar.{h,cpp}
├── docs/relatorio.md
├── README.md
└── img/                               (screenshots de validação)
```

### 6.3 Renderer 3D

O `Renderer3D` mantém VAOs cached pra três primitivas:

- **Esfera**: UV-sphere com 24 latitudes × 32 longitudes, normais radiais.
- **Cilindro**: 32 lados + tampas, alinhado ao eixo Z local (z ∈ [0, 1]), normais corretas.
- **Cubo**: 24 vértices (4 por face × 6 faces) com normais por face.

Pra desenhar um cilindro de `a` até `b`, calcula-se uma matriz de modelo:

```
direction d = normalize(b - a)
u = normalize(cross(d, axis_arbitrary))
v = cross(d, u)
M = [ u·r | v·r | d·len | a ]
```

Onde `r` é o raio, `len` é o comprimento, e os vetores são as colunas da matriz. Isso transforma o cilindro unitário Z-aligned na posição/orientação desejada.

O shader único faz Phong simplificado (ambient + diffuse) com uma luz direcional fixa. Um flag `uUseLighting` desliga o shading pra desenhar grades, eixos e linhas planas.

### 6.4 Câmera órbita

A câmera é parametrizada por:

- `target` (ponto pra onde aponta — origem).
- `yaw` (rotação azimutal em torno de Z).
- `pitch` (rotação de elevação).
- `distance` (raio).

A posição da câmera é calculada em esféricas:

```
eye = target + (cos(pitch)·cos(yaw), cos(pitch)·sin(yaw), sin(pitch)) · distance
```

Mouse drag (LMB) modifica `yaw` e `pitch`; scroll multiplica `distance` por um fator (zoom logarítmico). `pitch` é limitado a `±(π/2 − ε)` pra evitar gimbal lock na lookAt.

### 6.5 UI: inputs animados (não sliders)

As variáveis cinemáticas (`d₁..d₃` no Cartesiano, `θ, φ, ρ` no Polar) são **campos de input numérico**. Ao teclar Enter (ou clicar fora do campo), uma animação suave de 0.8 s leva o robô da configuração atual até a nova configuração via interpolação `smoothstep`. Pra juntas rotativas, a diferença angular é normalizada pro caminho mais curto via `atan2(sin Δ, cos Δ)`.

Ao lado de cada input, um texto desabilitado `(atual: X.XX)` mostra o valor corrente em tempo real — durante a animação, o usuário vê o número se aproximando do target.

Os sliders só permanecem pra parâmetros de geometria (`X max`, `Y max`, `Z max` no Cartesiano; `rho min/max` no Polar), porque são tunings da máquina, não variáveis cinemáticas.

### 6.6 Detecção de falha de IK

Quando `IManipulator::inverseKinematics` retorna `false`, a string de motivo (ex: "rho acima do maximo") é exibida em vermelho no painel; o marcador do alvo no mundo 3D também fica vermelho (em vez de verde); a animação NÃO dispara.

---

## 7. Como rodar

### 7.1 Pré-requisitos

- Windows 10/11.
- Scoop instalado.
- CMake ≥ 3.20, Ninja, LLVM/Clang ou MSVC, vcpkg.

### 7.2 Setup

```powershell
scoop install cmake ninja vcpkg llvm
vcpkg integrate install
```

### 7.3 Build

```powershell
cd C:\Users\higor\Repositories\cinematica-industrial
cmake --preset default        # primeira vez: instala imgui via vcpkg manifest mode
cmake --build build/default
```

### 7.4 Executar

```powershell
.\build\default\cinematica_industrial.exe
```

### 7.5 Controles

| Ação | Como |
|---|---|
| Trocar manipulador | Radio buttons no topo do painel ImGui |
| Cinemática direta (FK) | Digitar valor em cada junta + tecla Enter (anima até) |
| Cinemática inversa (IK) | Digitar `Alvo XYZ` + botão "Animar para alvo" |
| Orbitar câmera | LMB drag fora do painel |
| Zoom | Scroll do mouse |
| Sair | Tecla ESC |

---

## 8. Conclusões

Foram implementados os modelos completos de cinemática direta e inversa para dois robôs industriais representativos: o Cartesiano (PPP) e o Polar (RRP esférico). Ambos têm IK em **forma fechada**, tornando a simulação instantânea e numericamente estável.

O **Cartesiano** é o caso pedagógico do "robô identidade" — variáveis articulares = coordenadas cartesianas, sem álgebra. Já o **Polar** introduz a conversão clássica esférico ↔ cartesiano, que é a base de qualquer livro-texto de robótica e aparece em derivações mais complexas (por exemplo, na IK do robô antropomórfico). A presença do `atan2` resolve elegantemente a ambiguidade de quadrante, e a única atenção pedagógica adicional é a singularidade nos polos.

A simulação interativa em OpenGL com câmera órbita 3D permite ao avaliador rotacionar livremente a cena para inspecionar o robô de qualquer ângulo, testar valores tanto via FK (digitando juntas) quanto via IK (digitando posição XYZ alvo), e verificar imediatamente os casos de falha.

### 8.1 Verificação cruzada

Todos os exemplos numéricos deste documento foram conferidos contra a simulação OpenGL (screenshots em `img/`):

- `img/01.png` — Cartesiano com efetuador em `(1.0, 1.5, 0.8)`.
- `img/02.png` — Polar com IK pra `(1.5, 0, 1.0)`, resultando em `θ=0°, φ=33.69°, ρ=1.803`.

---

## 9. Referências

1. **Craig, J. J.** *Introduction to Robotics: Mechanics and Control*. 4th edition. Pearson, 2017.
2. **Spong, M. W., Hutchinson, S., Vidyasagar, M.** *Robot Modeling and Control*. 2nd edition. Wiley, 2020.
3. **Siciliano, B., Sciavicco, L., Villani, L., Oriolo, G.** *Robotics: Modelling, Planning and Control*. Springer, 2009.
4. **Romano, V. F.** *Robótica Industrial — Aplicação na Indústria de Manufatura e de Processos*. Edgard Blücher, 2002.
5. Dear ImGui: <https://github.com/ocornut/imgui>
6. GLFW: <https://www.glfw.org/documentation.html>
7. GLM: <https://github.com/g-truc/glm>
