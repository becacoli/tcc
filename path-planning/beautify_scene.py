"""Aplica esquemas de cores bonitos na cena do CoppeliaSim.

Aplica cor e configurações visuais em:
  - Chão (floor)
  - Paredes (todas que começam com wall_)
  - Goal (se existir)

Uso:
  python beautify_scene.py --theme wood       # paredes amadeiradas (marrom)
  python beautify_scene.py --theme purple     # paredes roxo forte
  python beautify_scene.py --theme nature     # verde com madeira
  python beautify_scene.py --theme dark       # estilo noturno
  python beautify_scene.py --theme list       # lista todos os temas

Pré-requisitos:
  - CoppeliaSim aberto com uma cena carregada
  - Simulação PARADA (botão stop ativo)
"""

import argparse
import sys


# ---------------------------------------------------------------------------
# Paleta de temas (cores em RGB normalizado: 0.0 a 1.0)
# ---------------------------------------------------------------------------
THEMES = {
    "wood": {
        "name":       "Madeira (Marrom + Chão Bege)",
        "description": "Paredes amadeiradas marrons, chão bege claro",
        "floor":          (0.92, 0.87, 0.78),   # bege claro
        "wall":           (0.55, 0.35, 0.20),   # marrom madeira
        "wall_specular":  (0.30, 0.20, 0.10),
        "wall_shininess": 25,
        "goal":           (0.95, 0.80, 0.20),   # amarelo dourado
    },
    "wood_black": {
        "name":       "Madeira (Marrom + Chão Preto)",
        "description": "Paredes amadeiradas marrons sobre chão preto — visual sofisticado",
        "floor":          (0.05, 0.05, 0.07),   # preto bem escuro
        "wall":           (0.65, 0.40, 0.22),   # marrom madeira mais quente
        "wall_specular":  (0.50, 0.30, 0.15),   # brilho dourado
        "wall_shininess": 40,
        "goal":           (1.00, 0.85, 0.15),   # amarelo dourado vibrante
    },
    "wood_full": {
        "name":       "Madeira Completa (Chão claro + Paredes escuras) ⭐",
        "description": "Chão de madeira clara + paredes de madeira escura — visual de cabana",
        "floor":          (0.78, 0.62, 0.40),   # madeira clara (pinho)
        "wall":           (0.42, 0.27, 0.15),   # madeira escura (mogno)
        "wall_specular":  (0.55, 0.35, 0.20),
        "wall_shininess": 30,
        "goal":           (1.00, 0.85, 0.15),
    },
    "wood_warm": {
        "name":       "Madeira Aquecida (Chão bege + Paredes castanhas) ⭐",
        "description": "Chão bege quentinho + paredes castanhas — visual aconchegante",
        "floor":          (0.88, 0.78, 0.60),   # bege quentinho
        "wall":           (0.55, 0.30, 0.15),   # castanho avermelhado
        "wall_specular":  (0.75, 0.45, 0.25),
        "wall_shininess": 45,
        "goal":           (0.95, 0.85, 0.20),
    },
    "purple": {
        "name":       "Roxo Forte (Estilo Imperial)",
        "description": "Paredes em roxo vibrante, chão cinza claro",
        "floor":          (0.93, 0.93, 0.95),   # branco azulado
        "wall":           (0.45, 0.20, 0.65),   # roxo forte
        "wall_specular":  (0.65, 0.40, 0.85),
        "wall_shininess": 60,
        "goal":           (1.00, 0.85, 0.10),   # amarelo dourado vibrante
    },
    "purple_yellow": {
        "name":       "Roxo + Amarelo (Lakers) ⭐",
        "description": "Chão roxo forte com paredes amarelas — alto contraste",
        "floor":          (0.40, 0.15, 0.60),   # roxo forte
        "wall":           (1.00, 0.85, 0.10),   # amarelo vibrante
        "wall_specular":  (1.00, 0.95, 0.40),   # brilho dourado
        "wall_shininess": 70,
        "goal":           (0.95, 0.30, 0.20),   # vermelho contrastante
    },
    "lavender": {
        "name":       "Lavanda (Roxo Suave)",
        "description": "Paredes em tom lavanda, chão branco",
        "floor":          (0.97, 0.97, 0.98),
        "wall":           (0.65, 0.55, 0.85),   # lavanda
        "wall_specular":  (0.80, 0.70, 0.95),
        "wall_shininess": 40,
        "goal":           (1.00, 0.75, 0.25),
    },
    "nature": {
        "name":       "Natureza (Verde + Madeira)",
        "description": "Paredes em madeira escura, chão verde grama clara",
        "floor":          (0.78, 0.88, 0.72),   # verde grama claro
        "wall":           (0.40, 0.27, 0.15),   # madeira escura
        "wall_specular":  (0.20, 0.15, 0.10),
        "wall_shininess": 20,
        "goal":           (0.95, 0.40, 0.10),   # laranja vibrante
    },
    "dark": {
        "name":       "Estilo Noturno (Escuro)",
        "description": "Paredes pretas com leve brilho, chão cinza escuro",
        "floor":          (0.25, 0.25, 0.28),
        "wall":           (0.10, 0.10, 0.15),
        "wall_specular":  (0.40, 0.40, 0.50),
        "wall_shininess": 80,
        "goal":           (0.30, 0.95, 0.45),   # verde neon
    },
    "academic": {
        "name":       "Acadêmico (Limpo e Profissional)",
        "description": "Paredes azul-acinzentado, chão branco — bom pra TCC",
        "floor":          (0.96, 0.96, 0.97),
        "wall":           (0.45, 0.55, 0.70),
        "wall_specular":  (0.60, 0.70, 0.85),
        "wall_shininess": 50,
        "goal":           (0.95, 0.30, 0.20),
    },
    "warm": {
        "name":       "Aconchegante (Tons Quentes)",
        "description": "Paredes terracota, chão bege claro",
        "floor":          (0.95, 0.90, 0.82),
        "wall":           (0.75, 0.40, 0.30),   # terracota
        "wall_specular":  (0.85, 0.55, 0.40),
        "wall_shininess": 35,
        "goal":           (0.20, 0.50, 0.85),   # azul contrastante
    },
    "pastel": {
        "name":       "Pastel (Suave)",
        "description": "Paredes rosa pastel, chão azul muito claro",
        "floor":          (0.92, 0.96, 0.98),
        "wall":           (0.90, 0.70, 0.75),
        "wall_specular":  (0.95, 0.85, 0.88),
        "wall_shininess": 45,
        "goal":           (0.30, 0.70, 0.60),
    },
}


# ---------------------------------------------------------------------------
# Aplicação dos temas
# ---------------------------------------------------------------------------

def list_themes():
    print("\nTemas disponíveis:\n")
    for key, theme in THEMES.items():
        print(f"  {key:<12} → {theme['name']}")
        print(f"  {'':<12}   {theme['description']}\n")


def remove_texture(sim, handle):
    """Remove a textura de um shape — necessário pra cor aparecer."""
    try:
        # Tenta limpar a textura (depende da versão do CoppeliaSim)
        sim.setShapeTexture(handle, -1, sim.texturemap_plane, 0, [0.0, 0.0])
        return True
    except Exception:
        try:
            # Versão alternativa
            sim.setShapeTexture(handle, -1, 0, 0, [0.0, 0.0])
            return True
        except Exception:
            return False


def get_all_shape_parts(sim, handle):
    """Retorna o handle + todos os shapes filhos (para compound shapes)."""
    parts = [handle]
    try:
        children = sim.getObjectsInTree(handle, sim.object_shape_type, 1)
        for c in children:
            if c != handle and c not in parts:
                parts.append(c)
    except Exception:
        pass
    return parts


def set_color(sim, handle, color_rgb, kind="ambient_diffuse"):
    """Define a cor de um shape no CoppeliaSim (e dos filhos, se houver)."""
    component = {
        "ambient_diffuse": sim.colorcomponent_ambient_diffuse,
        "specular":        sim.colorcomponent_specular,
        "emission":        sim.colorcomponent_emission,
    }.get(kind, sim.colorcomponent_ambient_diffuse)

    # Remove textura primeiro (senão a cor não aparece)
    if kind == "ambient_diffuse":
        remove_texture(sim, handle)

    # Aplica cor no shape principal E em todas as partes filhas
    success = False
    for h in get_all_shape_parts(sim, handle):
        try:
            if kind == "ambient_diffuse":
                remove_texture(sim, h)
            sim.setShapeColor(h, '', component, list(color_rgb))
            success = True
        except Exception:
            pass
    return success


def apply_theme(sim, theme_key):
    if theme_key not in THEMES:
        print(f"[ERRO] Tema '{theme_key}' não existe. Use --theme list para ver opções.")
        return False

    theme = THEMES[theme_key]
    print(f"\n🎨 Aplicando tema: {theme['name']}")
    print(f"   {theme['description']}\n")

    # ---- Coleta todos os shapes ----
    try:
        handles = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    except Exception as e:
        print(f"[ERRO] Não consegui listar objetos: {e}")
        return False

    floor_count = 0
    wall_count = 0
    goal_count = 0

    for h in handles:
        try:
            alias = sim.getObjectAlias(h)
        except Exception:
            continue
        if not alias:
            continue
        short = alias.split("/")[-1].lower()

        # CHÃO — remove textura xadrez e aplica cor
        if short == "floor" or "floor" in short.lower():
            # Remove textura agressivamente em todas as partes
            for part in get_all_shape_parts(sim, h):
                remove_texture(sim, part)
                try:
                    # Força transparência opaca pro caso de chão semi-transparente
                    sim.setObjectFloatParam(part, sim.shapefloatparam_init_velocity_a, 0.0)
                except Exception:
                    pass
            set_color(sim, h, theme["floor"], "ambient_diffuse")
            set_color(sim, h, (0.05, 0.05, 0.05), "specular")
            floor_count += 1
            print(f"  ✓ Chão: {alias} → cor aplicada (textura removida)")

        # PAREDES — incluindo todas as partes (compound shapes)
        elif short.startswith("wall_"):
            for part in get_all_shape_parts(sim, h):
                remove_texture(sim, part)
            set_color(sim, h, theme["wall"], "ambient_diffuse")
            set_color(sim, h, theme["wall_specular"], "specular")
            wall_count += 1

    print(f"\n  ✓ {wall_count} paredes coloridas")

    # GOAL (procura objeto Dummy chamado GoalConfiguration)
    try:
        goal_handle = sim.getObject('/GoalConfiguration')
        # Goal é um Dummy, não shape — não tem cor. Tentamos colorir os shapes filhos se houver.
        # Em geral o Goal é só um marcador (já vem amarelo)
        goal_count = 1
        print(f"  ✓ Goal detectado em /GoalConfiguration")
    except Exception:
        print(f"  [info] Goal não encontrado (ou já está visível)")

    # ---- Configurações visuais extras ----
    print("\n  Aplicando configurações visuais extras...")
    try:
        # Aumenta qualidade visual
        # 26: shadow flag, 27: anti-aliasing
        sim.setBoolParam(sim.boolparam_display_enabled, True)
        print("  ✓ Display habilitado")
    except Exception:
        pass

    print(f"\n🎉 Tema '{theme_key}' aplicado com sucesso!\n")
    print("   Próximo passo: vista superior (numpad 5) e tira screenshot.\n")
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Aplica temas visuais bonitos em cenas do CoppeliaSim.",
    )
    parser.add_argument(
        "--theme",
        default="wood",
        help="Tema a aplicar (use 'list' pra ver opções). "
             "Padrão: wood.",
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()

    if args.theme == "list":
        list_themes()
        return 0

    # Conecta ao CoppeliaSim
    print(f"Conectando ao CoppeliaSim em {args.host}:{args.port}...")
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        client = RemoteAPIClient(args.host, args.port)
        sim = client.getObject('sim')
        print("✓ Conectado.\n")
    except Exception as e:
        print(f"[ERRO] Não consegui conectar: {e}")
        print("Verifica se o CoppeliaSim está aberto.")
        return 1

    # Verifica se a simulação está parada
    try:
        state = sim.getSimulationState()
        if state != sim.simulation_stopped:
            print("⚠️  ATENÇÃO: a simulação está rodando.")
            print("   Recomendo parar antes (botão ■) para que as cores fiquem salvas.\n")
            resp = input("   Continuar mesmo assim? [s/N] ").strip().lower()
            if resp not in ("s", "sim", "y", "yes"):
                return 0
    except Exception:
        pass

    return 0 if apply_theme(sim, args.theme) else 1


if __name__ == "__main__":
    sys.exit(main())
