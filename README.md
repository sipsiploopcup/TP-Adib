# TP Lidar LD06 – Jour 1

Extraction, visualisation et cartographie (grid map et carte vectorielle) à partir de scans LD06.

## Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Enregistrer un scan réel

Dans le programme ESP32, utiliser `ld06.printScanCSV(Serial)` à la place
de `ld06.printScanTeleplot(Serial)`, puis téléverser. Fermer le Serial Monitor
Arduino et immobiliser le capteur pendant l'acquisition.

Depuis la racine du projet :

```bash
source .venv/bin/activate
python acquisition.py --list-ports
python acquisition.py --port /dev/cu.usbserial-1420
```

Adapter le port à celui affiché sur le poste. Par défaut, le programme enregistre
un scan dans `scans/`, puis s'arrête. Chaque fichier porte un nom horodaté et
conserve les six colonnes : `point_index,angle_deg,dist_mm,intensity,x_lib_mm,y_lib_mm`.
Les x/y de la bibliothèque sont conservés sans modification de leur repère.
Le script attend l'index zéro, des indices consécutifs, une couverture angulaire
de 355° au moins jusqu'à 5° au plus, et la ligne vide finale. Il refuse les blocs
mal formés. Ces contrôles ne garantissent pas l'absence de pertes UART dans l'ESP32.
Le CRC LD06 est contrôlé par la bibliothèque embarquée ; le flux CSV ne le transmet pas.

Noter séparément pour chaque fichier la position approximative du capteur dans
la salle : x et y en mètres, orientation en degrés, dans un repère commun.
Déplacer le capteur entre acquisitions et le laisser immobile pendant chaque scan.

Tests du lecteur (données synthétiques, sans matériel) :

```bash
python -m pytest tests/test_acquisition.py -q
```

## Afficher et placer les scans

```bash
python viewer.py scans/scan_20261102_124403_644384.csv
```

Passer plusieurs chemins CSV pour comparer plusieurs scans dans des couleurs
distinctes. Sélectionner le scan à droite, puis modifier X/Y (mètres) et θ
(degrés, sens antihoraire) avec les curseurs. « Recentrer » ajuste les limites
de la vue. « Enregistrer poses » sauvegarde les poses dans `poses.json`,
rechargé au prochain lancement. Les fichiers CSV restent inchangés.

Le visualiseur utilise les x/y déjà calculés par la bibliothèque : division
par 1000 pour passer en mètres et inversion de `y_lib_mm` pour suivre le
repère mathématique présenté dans le TP. Ce choix suppose les décalages
de pose embarqués laissés à zéro, comme dans le programme de test utilisé.
Les mesures de distance nulle sont exclues de l'affichage.

## Fichiers

| Fichier | Rôle |
|---|---|
| `acquisition.py` | Lecture du LD06 sur le port série, enregistrement des scans en CSV (polaire) |
| `scans/` | Scans réels acquis dans la salle |
| `geometry.py` | Conversion polaire → cartésien, application d'une pose |
| `viewer.py` | Affichage de plusieurs scans, réglage manuel de la pose de chacun |
| `gridmap.py` | Création et mise à jour d'une carte d'occupation |
| `segments.py` | Extraction de segments et opérations géométriques |
| `vectormap.py` | Construction de la carte vectorielle |
| `tests/` | Tests unitaires (`pytest`) |
| `RAPPORT.md` | Démarche, choix techniques, hypothèses, difficultés, limites |
