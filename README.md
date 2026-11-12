# A* Yol Bulma Algoritması — Sinematik Manim Görselleştirmesi

A* algoritmasının bir ızgara üzerinde en kısa yolu nasıl bulduğunu adım adım gösteren, tamamen Python koduyla üretilmiş 1 dakikalık bir animasyon.

![A* Yol Bulma tanıtımı](github_1280x720.mp4)

## Amaç

A* çoğu kaynakta sözde kodla anlatılır; `f = g + h` formülünün aramayı neden hedefe doğru "çektiği" ise ancak algoritma çalışırken görülünce oturur. Bu proje şunları görünür kılar:

- Açık kümenin (keşfedilen düğümler) ve kapalı kümenin (genişletilen düğümler) nasıl büyüdüğü
- Her hücrenin `f` değeri ve o an genişletilen düğümün `g`, `h` bileşenleri
- Bir duvar yolu kestiğinde aramanın nasıl geri yayıldığı
- Aynı haritada Dijkstra'ya göre kaç düğüm daha az tarandığı

Videodaki tüm sayılar render sırasında algoritmadan hesaplanır; hiçbiri elle yazılmamıştır.

## Teknolojiler

| Bileşen | Kullanım |
|---|---|
| Python 3.10+ | Algoritma ve sahne kodu |
| [Manim Community](https://www.manim.community/) | Animasyon motoru (`MovingCameraScene` ile zoom ve pan) |
| `heapq` | Öncelik kuyruğu (açık küme) |

LaTeX kurulumu gerekmez; tüm yazılar `Text` ile çizilir.

## Kurulum ve çalıştırma

```bash
pip install -r requirements.txt
manim -pql astar_cinematic.py AStarCinematic
```

`-pql` düşük kalitede (480p) hızlıca render alır ve videoyu açar. Paylaşım için:

```bash
manim -pqh astar_cinematic.py AStarCinematic   # 1080p, 60 fps
```

Çıktı `media/videos/astar_cinematic/` klasörüne yazılır.

Linux'ta `pip install manim` derleme hatası verirse Pango ve Cairo geliştirme paketleri eksiktir (Ubuntu: `sudo apt install libpango1.0-dev libcairo2-dev pkg-config`). Diğer sistemler için [resmi kurulum rehberine](https://docs.manim.community/en/stable/installation.html) bakın.

Manim Community 0.22.1 ve Python 3.12 ile test edilmiştir.

## Proje yapısı

```
.
├── astar_cinematic.py   # Algoritma + Manim sahnesi
├── requirements.txt
├── github_demo.gif      # README önizlemesi
└── README.md
```

`astar_cinematic.py` iki bölümden oluşur:

1. **Algoritma** (`parse_map`, `manhattan`, `a_star`): Manim'den bağımsız saf Python. `a_star` her genişletme adımını bir listeye kaydeder.
2. **Sahne** (`AStarCinematic`): Bu kaydı oynatır. Algoritma ile çizim ayrı olduğu için aynı kayıt başka bir görselleştirmede de kullanılabilir.

## Algoritma

A* her adımda açık kümeden `f(n) = g(n) + h(n)` değeri en küçük düğümü seçer.

- `g(n)`: başlangıçtan `n` düğümüne gerçek maliyet
- `h(n)`: `n` düğümünden hedefe tahmini maliyet. Burada 4 yönlü hareket için Manhattan uzaklığı: `|r1 - r2| + |c1 - c2|`

Manhattan uzaklığı bu ızgarada gerçek mesafeyi asla aşmaz (kabul edilebilir) ve tutarlıdır. Bu yüzden bulunan yol en kısadır ve kapalı kümeye giren bir düğüm tekrar açılmaz.

`f` değerleri eşit olduğunda `h` değeri küçük olan (hedefe daha yakın) düğüm önce genişletilir.

| | Zaman | Bellek |
|---|---|---|
| En kötü durum | O(E log V) | O(V) |

En kötü durum Dijkstra ile aynıdır; kazanç pratikte, sezgiselin aramayı daraltmasından gelir. Varsayılan haritada:

| Algoritma | Genişletilen düğüm | Yol uzunluğu |
|---|---|---|
| A* (Manhattan) | 58 | 19 |
| Dijkstra (`h = 0`) | 121 | 19 |

## Özelleştirme

Dosyanın başındaki ayarlar:

| Ayar | Açıklama |
|---|---|
| `GRID_MAP` | Harita. `S` başlangıç, `G` hedef, `#` duvar, `.` boş |
| `LANG` | Ekran yazıları: `"tr"` veya `"en"` |
| `AUTHOR` | Kapanış ekranındaki imza |
| `ZOOM`, `CLOSE_UP_STEPS` | Yakın planın oranı ve kaç adım süreceği |

Harita değiştiğinde istatistikler kendiliğinden güncellenir. Varsayılan kadraja en fazla 16 sütun ve 9 satır sığar; daha büyük haritalar için `CELL` değerini küçültün.

## Lisans

MIT

## Geliştiren

Koray · [github.com/koraycigdem](https://github.com/koraycigdem)
