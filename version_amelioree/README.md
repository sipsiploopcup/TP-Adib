# Version améliorée — TP 1, 2 et 3 (LiDAR LD06)

Ce dossier regroupe le code des trois TP dans une version améliorée, les
données nécessaires pour tout réexécuter, et les rapports mis à jour.
Le code d'origine reste inchangé à la racine du dépôt.

## Installation

```bash
cd version_amelioree
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest -q          # 98 tests
```

Toutes les commandes s'exécutent depuis ce dossier. Les sorties sont écrites
dans `resultats/` (non versionné).

## Organisation

| Élément | Contenu |
|---|---|
| `donnees/` | 12 scans bruts CSV (`scans/`), classeurs Excel Scan 1/2/3/fusion, poses de référence estimées hors ligne (`alignements.json`) |
| TP 1 | `acquisition.py`, `geometry.py`, `viewer.py`, `merge_scans.py`, `gridmap.py`, `segments.py`, `vectormap.py` |
| TP 2 | `localisation_grid.py`, `localisation_vectorielle.py`, `localisation_scans.py`, `localisation_paires.py`, `comparaison_localisation.py` |
| TP 3 | `slam.py`, `slam_robot.py`, `slam_tp3.py` (SLAM principal), `slam_carte.py` et `robot_wifi.py` (variante scan-sur-carte), `compare_tp3.py` (banc d'essai) |
| `figures_ameliorees.py` | Figure de l'artefact d'échantillonnage (rapport TP 2) |
| `rapports/` | Rapports LaTeX et PDF des TP 1, 2 et 3 |
| `tests/` | Tests Pytest des trois TP |

## Améliorations

### TP 1 — cartographie
- **Tracé des rayons vectorisé** (`gridmap.trace_rays`) : même algorithme de
  Bresenham exécuté sur tous les rayons à la fois. Cartes identiques bit à bit
  (test d'équivalence), mise à jour environ 3 fois plus rapide.

### TP 2 — localisation
- **Distance point-à-ligne** pour la recherche entre scans : écart le long de
  la normale du mur local, repli point-à-point loin d'un mur.
- **ICP en deux phases** : point-à-point (robuste) puis point-à-ligne (précis),
  seconde phase rejetée si elle dégrade le coût. En simulation, l'erreur
  angulaire passe de 0,277° à 0,111°.
- **Artefact d'échantillonnage démontré** : sur les points réels réservés, la
  pose vraie est un maximum local du coût ; les ~0,8° d'erreur des méthodes à
  base de points viennent du protocole, pas des méthodes.

### TP 3 — SLAM et exploration
- Navigation à l'estime quand un scan est rejeté ; affinage de chaque pose sur
  la grille, validé contre la sous-carte ICP.
- Optimisation du graphe vectorisée à jacobienne creuse ; carte reconstruite
  depuis les images clés et seulement si la correction le justifie.
- Exploration : frontières qui ne traversent plus les murs, frontière la plus
  proche, abandon des frontières inatteignables, visée en ligne de vue,
  arrêt d'urgence au rayon du robot.
- Robot réel : repli automatique vers `/scan` du firmware RobotControl
  d'origine, pose SLAM affichée dans RobotControl, commande calibrée utilisée
  comme prédiction.

Les chiffres avant/après sont dans les rapports.

## Commandes principales

```bash
# TP 1
python gridmap.py "donnees/Scan 1.xlsx" --no-show --output resultats/carte_scan_1
python vectormap.py "donnees/scan fusion.xlsx" --no-show --output resultats/vecteurs_fusion

# TP 2
python comparaison_localisation.py --scans "donnees/Scan 1.xlsx" "donnees/scan 2.xlsx" "donnees/scan 3.xlsx"
python figures_ameliorees.py

# TP 3 : rejeu des 12 scans réels, simulation, banc comparatif
python slam_tp3.py --mode replay --wide-search
python slam_tp3.py --mode simulate --show
python compare_tp3.py

# TP 3 : robot réel (PC connecté au Wi-Fi RobotESP32)
python slam_tp3.py --mode robot --steps 20                      # cartographie, pilotage manuel
python robot_wifi.py --calibrer --auto                          # mesure vitesse et rotation
python slam_tp3.py --mode robot --auto --speed 0.25 --turn-rate 90
```

Le mode robot n'a été testé qu'avec un ESP32 simulé ; vérifier d'abord le
mode manuel, puis la calibration, avant toute exploration automatique.
