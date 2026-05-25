# cinematica-industrial

Simulação 3D OpenGL de **dois robôs industriais** — Cartesiano (PPP) e Polar/Esférico (RRP) — com cinemática direta e inversa interativas. Trabalho da disciplina de Cinemática.

> **Documento técnico completo:** [`docs/relatorio.md`](docs/relatorio.md) — classificação dos robôs industriais, derivação de FK e IK, exemplos numéricos passo-a-passo, implementação.

## Demo

| Cartesiano (PPP) | Polar / Esférico (RRP) |
|---|---|
| ![Cartesiano](img/01.png) | ![Polar](img/02.png) |

## Robôs implementados

| Robô | Tipo | Juntas | FK | IK |
|---|---|---|---|---|
| **Cartesiano** | PPP | d₁ (X), d₂ (Y), d₃ (Z) | x=d₁, y=d₂, z=d₃ | d₁=x, d₂=y, d₃=z |
| **Polar (Esférico)** | RRP | θ (azimute), φ (elevação), ρ (radial) | x=ρcosφcosθ, y=ρcosφsinθ, z=ρsinφ | ρ=√(x²+y²+z²), θ=atan2(y,x), φ=atan2(z, √(x²+y²)) |

## Dependências

- Windows 10/11
- CMake ≥ 3.20, Ninja
- LLVM/Clang ou MSVC
- vcpkg (instala imgui/glfw/glad/glm automaticamente via `vcpkg.json` em modo manifest)

## Build

```powershell
cmake --preset default          # primeira vez: ~1 min (vcpkg compila imgui)
cmake --build build/default
```

## Executar

```powershell
.\build\default\cinematica_industrial.exe
```

## Controles

| Ação | Como |
|---|---|
| Trocar de robô | Radio buttons "Cartesiano (PPP)" / "Polar (RRP esferico)" |
| Cinemática direta (FK) | Digitar valor em cada junta + tecla Enter (anima até) |
| Cinemática inversa (IK) | Digitar `Alvo XYZ` + botão "Animar para alvo" |
| **Orbitar câmera** | **LMB drag fora do painel** |
| **Zoom** | **Scroll do mouse** |
| Sair | Tecla ESC |

## Convenções

- **Z = pra cima** (convenção robótica). O piso/grid fica no plano XY.
- Eixos coloridos: **X = vermelho**, **Y = verde**, **Z = azul**.
- Ângulos digitados em **graus** na UI, armazenados em radianos internamente.

## Indicadores visuais

| Elemento | Significado |
|---|---|
| Esfera laranja na origem | Base do robô |
| Esfera rosa | Efetuador |
| Cilindro cinza/prata | Junta prismática (cilindro hidráulico) |
| Cubo laranja | Indicador de junta prismática |
| Wireframe azul-claro | Workspace (caixa pro Cartesiano, anel pro Polar) |
| Cruz verde no alvo | IK ok |
| Cruz vermelha no alvo | IK falhou (mensagem detalhada no painel) |
