"""
A* Yol Bulma Algoritması - Sinematik Manim Görselleştirmesi
===========================================================

Çalıştırma (hızlı önizleme):
    manim -pql astar_cinematic.py AStarCinematic

Yüksek kalite (1080p, 60 fps):
    manim -pqh astar_cinematic.py AStarCinematic

Notlar:
- LaTeX GEREKTİRMEZ: tüm yazılar Text (Pango) ile çizilir.
- Haritayı değiştirmek için aşağıdaki GRID_MAP listesini düzenlemek yeterlidir.
- Ekrandaki istatistikler (genişletilen düğüm sayısı, yol uzunluğu, Dijkstra
  karşılaştırması) sabit yazılmamıştır; her render'da algoritmadan hesaplanır.
"""

import heapq
import itertools

from manim import *

# --------------------------------------------------------------------------
# AYARLAR
# --------------------------------------------------------------------------

LANG = "tr"  # Ekran yazılarının dili: "tr" veya "en"
AUTHOR = "github.com/koraycigdem"  # Kapanış ekranında görünen imza

# Harita:  S = başlangıç, G = hedef, # = duvar, . = boş hücre
# Tüm satırlar aynı uzunlukta olmalıdır.
GRID_MAP = [
    "................",
    "......#.........",
    "......#...###...",
    "......#.....#...",
    ".S....#.....#.G.",
    "......#.....#...",
    "...####...###...",
    "................",
    "................",
]

CELL = 0.6          # Bir hücrenin kenar uzunluğu (sahne birimi)
ZOOM = 0.45         # Arama başlarken kameranın yakınlaşma oranı
CLOSE_UP_STEPS = 14  # Kamera kaç adım boyunca yakın planda kalsın

# Renk paleti (GitHub koyu temasına yakın)
BG = "#0d1117"
C_EMPTY = "#161b22"
C_STROKE = "#30363d"
C_WALL = "#6e7681"
C_OPEN = "#1f6feb"     # Açık küme: keşfedildi, henüz genişletilmedi
C_CLOSED = "#6e40c9"   # Kapalı küme: genişletildi
C_START = "#3fb950"
C_GOAL = "#f85149"
C_PATH = "#f2cc60"
C_TEXT = "#e6edf3"
C_MUTED = "#8b949e"

TEXTS = {
    "tr": {
        "title": "A* Yol Bulma Algoritması",
        "subtitle": "En kısa yolu, tüm haritayı taramadan bulmak",
        "g_desc": "g(n): başlangıçtan n'e gelmenin gerçek maliyeti",
        "h_desc": "h(n): n'den hedefe tahmini uzaklık (Manhattan)",
        "cap_grid": "Izgara bir graftır: her hücre bir düğüm, her komşuluk 1 birimlik bir kenar.",
        "cap_rule": "Kural: açık kümeden her zaman f = g + h değeri en küçük olan düğümü genişlet.",
        "cap_path": "Hedefe ulaşıldı. Yol, ebeveyn işaretçileri geriye doğru izlenerek kurulur.",
        "leg_start": "Başlangıç",
        "leg_goal": "Hedef",
        "leg_wall": "Duvar",
        "leg_open": "Açık küme",
        "leg_closed": "Kapalı küme",
        "leg_path": "En kısa yol",
        "hud_step": "Adım",
        "hud_open": "Açık",
        "hud_closed": "Kapalı",
        "stat_title": "Genişletilen düğüm",
        "stat_len": "Yol uzunluğu: {n} adım",
        "stat_saving": "A*, %{p} daha az düğüm taradı",
        "outro_1": "A* = Dijkstra + iyi bir sezgisel",
        "outro_2": "Sezgisel hedefe olan uzaklığı asla abartmıyorsa, bulunan yol en kısadır.",
    },
    "en": {
        "title": "A* Pathfinding Algorithm",
        "subtitle": "Finding the shortest path without scanning the whole map",
        "g_desc": "g(n): actual cost from the start to n",
        "h_desc": "h(n): estimated distance from n to the goal (Manhattan)",
        "cap_grid": "The grid is a graph: every cell is a node, every adjacency an edge of cost 1.",
        "cap_rule": "Rule: always expand the open node with the smallest f = g + h.",
        "cap_path": "Goal reached. The path is rebuilt by walking the parent pointers backwards.",
        "leg_start": "Start",
        "leg_goal": "Goal",
        "leg_wall": "Wall",
        "leg_open": "Open set",
        "leg_closed": "Closed set",
        "leg_path": "Shortest path",
        "hud_step": "Step",
        "hud_open": "Open",
        "hud_closed": "Closed",
        "stat_title": "Nodes expanded",
        "stat_len": "Path length: {n} steps",
        "stat_saving": "A* expanded {p}% fewer nodes",
        "outro_1": "A* = Dijkstra + a good heuristic",
        "outro_2": "If the heuristic never overestimates the distance, the path is optimal.",
    },
}
T = TEXTS[LANG]


# --------------------------------------------------------------------------
# ALGORİTMA (Manim'den bağımsız, saf Python)
# --------------------------------------------------------------------------

def parse_map(grid_map):
    """Metin haritayı (satır, sütun) koordinatlarına çevirir."""
    rows, cols = len(grid_map), len(grid_map[0])
    if any(len(line) != cols for line in grid_map):
        raise ValueError("GRID_MAP satırlarının hepsi aynı uzunlukta olmalı.")
    walls, start, goal = set(), None, None
    for r, line in enumerate(grid_map):
        for c, ch in enumerate(line):
            if ch == "#":
                walls.add((r, c))
            elif ch == "S":
                start = (r, c)
            elif ch == "G":
                goal = (r, c)
    if start is None or goal is None:
        raise ValueError("Haritada bir 'S' (başlangıç) ve bir 'G' (hedef) olmalı.")
    return rows, cols, walls, start, goal


def manhattan(a, b):
    """4 yönlü ızgarada kabul edilebilir (admissible) ve tutarlı sezgisel."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def zero_heuristic(a, b):
    """h = 0 verildiğinde A*, Dijkstra algoritmasına indirgenir."""
    return 0


def a_star(rows, cols, walls, start, goal, heuristic=manhattan):
    """
    A* aramasını çalıştırır ve animasyon için adım adım kayıt döndürür.

    Dönüş:
        steps: [(genişletilen_düğüm, g, h, [(yeni/iyileşen komşu, f), ...]), ...]
        path : başlangıçtan hedefe düğüm listesi (yol yoksa boş liste)
    """
    counter = itertools.count()  # Eşitlik durumunda kararlı sıralama için
    g_score = {start: 0}
    parent = {}
    closed = set()
    # Öncelik: (f, h, sıra). f eşitse hedefe daha yakın olan (küçük h) önce gelir.
    h0 = heuristic(start, goal)
    heap = [(h0, h0, next(counter), start)]
    steps = []

    while heap:
        _, _, _, current = heapq.heappop(heap)
        if current in closed:
            continue  # Yığında kalmış eski (bayat) kayıt
        closed.add(current)
        g = g_score[current]

        if current == goal:
            steps.append((current, g, 0, []))
            path = [current]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return steps, path[::-1]

        opened = []
        r, c = current
        for nxt in ((r - 1, c), (r, c + 1), (r + 1, c), (r, c - 1)):
            nr, nc = nxt
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if nxt in walls or nxt in closed:
                continue
            tentative = g + 1
            if tentative < g_score.get(nxt, float("inf")):
                g_score[nxt] = tentative
                parent[nxt] = current
                h = heuristic(nxt, goal)
                heapq.heappush(heap, (tentative + h, h, next(counter), nxt))
                opened.append((nxt, tentative + h))
        steps.append((current, g, heuristic(current, goal), opened))

    return steps, []


# --------------------------------------------------------------------------
# SAHNE
# --------------------------------------------------------------------------

class AStarCinematic(MovingCameraScene):
    def construct(self):
        self.camera.background_color = BG
        self.frame = self.camera.frame
        self.frame.save_state()  # Geniş plana dönmek için ilk kamera durumu

        # Algoritmayı önce çalıştır; animasyon bu kayıtları oynatır.
        self.rows, self.cols, self.walls, self.start, self.goal = parse_map(GRID_MAP)
        self.steps, self.path = a_star(
            self.rows, self.cols, self.walls, self.start, self.goal
        )
        if not self.path:
            raise ValueError("Bu haritada başlangıçtan hedefe yol yok.")
        dijkstra_steps, _ = a_star(
            self.rows, self.cols, self.walls, self.start, self.goal, zero_heuristic
        )
        self.dijkstra_count = len(dijkstra_steps)

        self.intro()
        self.build_grid()
        self.search()
        self.reveal_path()
        self.show_stats()
        self.outro()

    # ---------------------------- yardımcılar -----------------------------

    def cell_pos(self, node):
        """(satır, sütun) -> sahne koordinatı."""
        r, c = node
        x = (c - (self.cols - 1) / 2) * CELL
        y = ((self.rows - 1) / 2 - r) * CELL
        return self.grid_origin + np.array([x, y, 0.0])

    def set_caption(self, text):
        """Izgaranın altındaki açıklama satırını değiştirir."""
        new = Text(text, font_size=22, color=C_TEXT)
        new.scale_to_fit_width(min(new.width, 12.5))
        new.move_to(DOWN * 3.55)
        if self.caption is None:
            self.play(FadeIn(new, shift=UP * 0.15), run_time=0.6)
        else:
            self.play(FadeOut(self.caption, shift=UP * 0.15),
                      FadeIn(new, shift=UP * 0.15), run_time=0.6)
        self.caption = new

    def clear_caption(self):
        if self.caption is not None:
            self.play(FadeOut(self.caption), run_time=0.4)
            self.caption = None

    def pin_to_camera(self, mobj, offset):
        """
        Nesneyi kameranın sol üst köşesine sabitler (HUD etkisi).
        Kamera yakınlaşıp kaydıkça nesne de aynı oranda ölçeklenir ve taşınır,
        böylece ekranda hep aynı yerde ve aynı boyutta görünür.
        """
        mobj.cam_scale = 1.0

        def follow(m):
            s = self.frame.width / config.frame_width
            m.scale(s / m.cam_scale)
            m.cam_scale = s
            corner = self.frame.get_corner(UL) + np.array([offset * s, -offset * s, 0])
            m.move_to(corner, aligned_edge=UL)

        mobj.add_updater(follow)

    def update_hud(self, step, n_open, n_closed, g, h):
        """HUD panelindeki sayıları yeniler."""
        s = self.hud.cam_scale
        lines = VGroup(
            Text(f"{T['hud_step']}: {step}", font_size=18, color=C_TEXT),
            Text(f"{T['hud_open']}: {n_open}   {T['hud_closed']}: {n_closed}",
                 font_size=18, color=C_MUTED),
            Text(f"f = g + h = {g} + {h} = {g + h}", font_size=18, color=C_PATH),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.14)
        lines.scale(s)
        lines.move_to(self.hud_bg.get_left() + RIGHT * 0.22 * s, aligned_edge=LEFT)
        lines.set_z_index(11)
        self.hud.remove(self.hud_lines)
        self.hud.add(lines)
        self.hud_lines = lines

    # ------------------------------ 1. giriş ------------------------------

    def intro(self):
        title = Text(T["title"], font_size=52, weight=BOLD, color=C_TEXT)
        subtitle = Text(T["subtitle"], font_size=26, color=C_MUTED)
        formula = Text(
            "f(n) = g(n) + h(n)", font_size=44, color=C_TEXT,
            t2c={"f(n)": C_PATH, "g(n)": C_START, "h(n)": C_GOAL},
        )
        g_desc = Text(T["g_desc"], font_size=24, color=C_START)
        h_desc = Text(T["h_desc"], font_size=24, color=C_GOAL)

        head = VGroup(title, subtitle).arrange(DOWN, buff=0.3)
        self.play(Write(title), run_time=1.4)
        self.play(FadeIn(subtitle, shift=UP * 0.2), run_time=0.8)
        self.wait(0.8)

        body = VGroup(formula, g_desc, h_desc).arrange(DOWN, buff=0.35)
        self.play(head.animate.scale(0.6).to_edge(UP, buff=0.6), run_time=0.8)
        self.play(FadeIn(formula, scale=0.9), run_time=0.8)
        self.play(FadeIn(g_desc, shift=RIGHT * 0.3), run_time=0.6)
        self.play(FadeIn(h_desc, shift=RIGHT * 0.3), run_time=0.6)
        self.wait(2)
        self.play(FadeOut(head), FadeOut(body), run_time=0.8)

    # --------------------------- 2. ızgara kurulumu ------------------------

    def build_grid(self):
        self.caption = None
        # Sol tarafta HUD ve lejant için yer bırakmak üzere ızgara sağa kaydırılır.
        self.grid_origin = RIGHT * 1.3 + DOWN * 0.1

        self.cells = {}
        for r in range(self.rows):
            for c in range(self.cols):
                sq = Square(side_length=CELL * 0.92)
                sq.set_stroke(C_STROKE, width=1.5)
                sq.set_fill(C_EMPTY, opacity=1)
                sq.move_to(self.cell_pos((r, c)))
                self.cells[(r, c)] = sq

        # Hücreler sol üstten sağ alta doğru köşegen bir dalga halinde belirir.
        ordered = sorted(self.cells, key=lambda n: (n[0] + n[1], n[0]))
        self.play(
            LaggedStart(*[FadeIn(self.cells[n], scale=0.6) for n in ordered],
                        lag_ratio=0.01),
            run_time=1.8,
        )

        wall_anims = [self.cells[w].animate.set_fill(C_WALL, 1).set_stroke(C_WALL)
                      for w in sorted(self.walls)]
        self.play(LaggedStart(*wall_anims, lag_ratio=0.05), run_time=1.2)

        self.cells[self.start].set_z_index(2)
        self.cells[self.goal].set_z_index(2)
        self.play(
            self.cells[self.start].animate.set_fill(C_START, 1).set_stroke(C_START),
            self.cells[self.goal].animate.set_fill(C_GOAL, 1).set_stroke(C_GOAL),
            run_time=0.6,
        )
        self.play(
            Indicate(self.cells[self.start], color=C_START, scale_factor=1.5),
            Indicate(self.cells[self.goal], color=C_GOAL, scale_factor=1.5),
            run_time=1.0,
        )

        # Lejant (sol sütun)
        entries = [
            (C_START, T["leg_start"]), (C_GOAL, T["leg_goal"]), (C_WALL, T["leg_wall"]),
            (C_OPEN, T["leg_open"]), (C_CLOSED, T["leg_closed"]), (C_PATH, T["leg_path"]),
        ]
        rows = VGroup()
        for color, label in entries:
            swatch = Square(side_length=0.22).set_fill(color, 1).set_stroke(width=0)
            text = Text(label, font_size=18, color=C_MUTED)
            rows.add(VGroup(swatch, text).arrange(RIGHT, buff=0.18))
        rows.arrange(DOWN, aligned_edge=LEFT, buff=0.18)
        rows.move_to(np.array([-6.75, -0.9, 0]), aligned_edge=LEFT)
        self.legend = rows
        self.play(FadeIn(self.legend, shift=RIGHT * 0.2), run_time=0.8)

        self.set_caption(T["cap_grid"])
        self.wait(2.2)
        self.set_caption(T["cap_rule"])
        self.wait(2.5)

    # ------------------------------ 3. arama ------------------------------

    def search(self):
        # HUD paneli: kameraya sabitlenir, yakın planda da okunur kalır.
        self.hud_bg = RoundedRectangle(width=3.7, height=1.35, corner_radius=0.12)
        self.hud_bg.set_fill("#010409", opacity=0.92).set_stroke(C_STROKE, width=1.5)
        self.hud_bg.set_z_index(10)
        self.hud_lines = VGroup()
        self.hud = VGroup(self.hud_bg, self.hud_lines)
        self.pin_to_camera(self.hud, offset=0.3)
        self.update_hud(0, 1, 0, 0, manhattan(self.start, self.goal))
        self.hud.update()
        self.play(FadeIn(self.hud), run_time=0.6)

        # İmleç: o an genişletilen düğümü gösteren çerçeve
        cursor = Square(side_length=CELL * 1.02).set_stroke(WHITE, width=4)
        cursor.set_fill(opacity=0).set_z_index(5)
        cursor.move_to(self.cell_pos(self.start))

        # Sinematik giriş: kamera başlangıç düğümüne yakınlaşır.
        self.clear_caption()
        self.play(
            self.frame.animate.scale(ZOOM).move_to(self.cell_pos(self.start)),
            FadeIn(cursor, scale=1.6),
            FadeOut(self.legend),  # Yakın planda lejant kadraja taşmasın
            run_time=2.0, rate_func=smooth,
        )

        labels = {}       # düğüm -> hücredeki f değeri yazısı
        open_set = {self.start}
        closed_count = 0

        for i, (current, g, h, opened) in enumerate(self.steps, start=1):
            close_up = i <= CLOSE_UP_STEPS
            # İlk adımlar yavaş (anlaşılır), sonrakiler giderek hızlanır.
            run_time = max(0.13, 0.75 * (0.88 ** (i - 1)))

            open_set.discard(current)
            closed_count += 1
            open_set.update(n for n, _ in opened)
            self.update_hud(i, len(open_set), closed_count, g, h)

            anims = [cursor.animate.move_to(self.cell_pos(current))]
            if current not in (self.start, self.goal):
                anims.append(self.cells[current].animate.set_fill(C_CLOSED, 0.9))

            for node, f in opened:
                if node != self.goal:
                    anims.append(self.cells[node].animate.set_fill(C_OPEN, 0.9))
                    new_label = Text(str(f), font_size=15, color=WHITE)
                    new_label.move_to(self.cell_pos(node)).set_z_index(3)
                    if node in labels:  # Daha iyi bir yol bulundu: f güncellenir
                        anims.append(Transform(labels[node], new_label))
                    else:
                        labels[node] = new_label
                        anims.append(FadeIn(new_label, scale=0.5))

            if close_up:
                # Kamera, genişletilen düğümü yumuşak bir pan ile izler.
                target = interpolate(self.frame.get_center(),
                                     self.cell_pos(current), 0.55)
                anims.append(self.frame.animate.move_to(target))

            self.play(*anims, run_time=run_time, rate_func=smooth)

            if i == CLOSE_UP_STEPS:
                # Geniş plana dönüş: aramanın hedefe doğru nasıl aktığı görülür.
                self.play(Restore(self.frame), FadeIn(self.legend),
                          run_time=1.8, rate_func=smooth)
                self.frame.save_state()

        if len(self.steps) < CLOSE_UP_STEPS:  # Çok kısa haritalar için güvenlik
            self.play(Restore(self.frame), FadeIn(self.legend), run_time=1.2)
            self.frame.save_state()

        self.play(
            Flash(self.cells[self.goal], color=C_GOAL, line_length=0.35,
                  num_lines=14, flash_radius=0.5),
            FadeOut(cursor),
            run_time=0.9,
        )
        self.labels = labels

    # --------------------------- 4. yolun çizilmesi ------------------------

    def reveal_path(self):
        self.set_caption(T["cap_path"])
        self.wait(1.0)

        points = [self.cell_pos(n) for n in self.path]
        line = VMobject().set_points_as_corners(points)
        line.set_stroke(C_PATH, width=7).set_z_index(6)
        glow = line.copy().set_stroke(C_PATH, width=18, opacity=0.25).set_z_index(5)
        dot = Dot(points[0], radius=0.13, color=WHITE).set_z_index(7)

        # Kamera başlangıca iner ve yol boyunca ilerleyen noktayı takip eder.
        self.clear_caption()
        self.play(
            self.frame.animate.scale(0.55).move_to(points[0]),
            FadeOut(VGroup(*self.labels.values())),
            FadeOut(self.legend),
            FadeIn(dot, scale=2),
            run_time=1.5,
        )
        self.frame.add_updater(lambda m: m.move_to(dot.get_center()))
        self.play(
            Create(line), Create(glow), MoveAlongPath(dot, line.copy()),
            run_time=5, rate_func=linear,
        )
        self.frame.clear_updaters()

        path_cells = [self.cells[n] for n in self.path[1:-1]]
        self.play(
            Restore(self.frame),
            LaggedStart(*[c.animate.set_fill(C_PATH, 0.55) for c in path_cells],
                        lag_ratio=0.08),
            FadeOut(dot),
            run_time=2.0,
        )
        self.path_line = VGroup(line, glow)

    # ----------------------------- 5. istatistik ---------------------------

    def show_stats(self):
        a_count = len(self.steps)
        d_count = self.dijkstra_count
        saving = round(100 * (1 - a_count / d_count))

        self.hud.clear_updaters()
        self.play(FadeOut(self.hud), run_time=0.6)

        # Sol sütunda A* ile Dijkstra'nın genişlettiği düğüm sayıları
        title = Text(T["stat_title"], font_size=20, color=C_TEXT)
        max_w = 2.4

        def bar(name, value, color):
            label = Text(f"{name}: {value}", font_size=18, color=color)
            rect = Rectangle(width=max_w * value / max(a_count, d_count), height=0.2)
            rect.set_fill(color, 1).set_stroke(width=0)
            return VGroup(label, rect).arrange(DOWN, aligned_edge=LEFT, buff=0.1)

        bar_a = bar("A*", a_count, C_PATH)
        bar_d = bar("Dijkstra", d_count, C_MUTED)
        length = Text(T["stat_len"].format(n=len(self.path) - 1),
                      font_size=18, color=C_TEXT)
        stats = VGroup(title, bar_a, bar_d, length)
        stats.arrange(DOWN, aligned_edge=LEFT, buff=0.3)
        stats.move_to(np.array([-6.8, 0.6, 0]), aligned_edge=LEFT)

        self.play(FadeIn(title), FadeIn(bar_a[0]), FadeIn(bar_d[0]), run_time=0.6)
        self.play(GrowFromEdge(bar_a[1], LEFT), GrowFromEdge(bar_d[1], LEFT),
                  run_time=1.2)
        self.play(FadeIn(length, shift=UP * 0.1), run_time=0.5)
        if saving > 0:
            self.set_caption(T["stat_saving"].format(p=saving))
        self.wait(3)
        self.stats = stats

    # ------------------------------ 6. kapanış -----------------------------

    def outro(self):
        everything = VGroup(*self.cells.values(), self.path_line, self.stats)
        fades = [FadeOut(everything)]
        if self.caption is not None:
            fades.append(FadeOut(self.caption))
        self.play(*fades, run_time=1.0)

        line_1 = Text(T["outro_1"], font_size=44, weight=BOLD, color=C_TEXT,
                      t2c={"A*": C_PATH})
        line_2 = Text(T["outro_2"], font_size=22, color=C_MUTED)
        sign = Text(AUTHOR, font_size=20, color=C_MUTED)
        VGroup(line_1, line_2).arrange(DOWN, buff=0.4)
        sign.to_edge(DOWN, buff=0.6)

        self.play(Write(line_1), run_time=1.4)
        self.play(FadeIn(line_2, shift=UP * 0.2), FadeIn(sign), run_time=0.8)
        self.wait(3)
        self.play(FadeOut(line_1), FadeOut(line_2), FadeOut(sign), run_time=1.0)
