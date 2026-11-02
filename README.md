# TP Lidar LD06 – Jour 1

Extraction, visualisation et cartographie (grid map et carte vectorielle) à partir de scans LD06.

## Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Enregistrer un scan réel

Teleplot a été utilisé pour la visualisation demandée par le professeur,
puis la sortie **CSV** a été remise sur l'ESP32 pour les acquisitions.
Les 12 scans enregistrés sont disponibles dans [scans/](./scans/).
Après toute nouvelle utilisation de Teleplot, remettre la sortie CSV sur
l'ESP32 avant d'utiliser le lecteur ci-dessous.

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

## Visualisation en direct avec Teleplot

1. Ouvrir [le programme fourni](./docs/ESP32_LD06_Test.ino) dans Arduino IDE.
2. Conserver `ld06.printScanTeleplot(Serial);`, sélectionner **ESP32 Dev Module**
   et téléverser à **115200 bauds**.
3. Fermer le Serial Monitor et le Serial Plotter d'Arduino.
4. Installer l'extension VS Code **Teleplot** (`alexnesnes.teleplot`).
5. Ouvrir la palette de commandes, choisir **teleplot: Start teleplot session**,
   sélectionner le port USB de l'ESP32 et **115200 bauds**, puis connecter.

L'affichage en direct a été confirmé par l'étudiant. La sortie Teleplot
contient des x/y, contrairement au CSV qui conserve aussi angle, distance
et intensité. Une seule application peut utiliser le port série à la fois.

Configuration utilisée : Arduino IDE 2.3.8, cœur ESP32 3.3.12, LD06 1.0.0,
Teleplot pour VS Code 1.1.4. Le programme Wi-Fi
[ESP32RobotControl.ino](./docs/ESP32RobotControl.ino) a également été compilé
et téléversé avec Async TCP 3.5.0 et ESP Async WebServer 3.12.1, avant retour
à Teleplot. Son affichage Wi-Fi n'a pas été validé.

## Ressources du TP

[docs/](./docs/) contient les documents, programmes, bibliothèques et archives
ZIP du TP. Les métadonnées Git imbriquées et les fichiers système macOS ont
été exclus, y compris dans les copies des ZIP ; les originaux téléchargés
restent inchangés. [CODEX_HANDOFF.md](./CODEX_HANDOFF.md) précise l'avancement
et les étapes restantes.

## Fichiers

### Fusion des scans enregistrés

```bash
python merge_scans.py scans/scan_*.csv --reference scans/scan_20261102_170438_115290.csv --output resultats --show
```

Le script estime les poses par ICP avec plusieurs rotations/translations
initiales, puis réévalue les scans difficiles sur l'union des scans mieux
alignés. Il génère un nuage fusionné CSV, une image et les poses et indicateurs
d'alignement dans `resultats/`. À la demande de l'étudiant, ces résultats
restent locaux et sont exclus de Git ; la commande ci-dessus les régénère.
Les déplacements n'ayant pas été mesurés, ces poses sont estimées et doivent
être contrôlées, notamment dans une pièce présentant des symétries.

Les points nuls sont exclus ; tous les autres points sont conservés, avec
leur scan source. Aucune interpolation de mur n'est ajoutée. Pour tracer
le résultat dans Excel, utiliser `x_m` et `y_m` (colonnes H et I).
Ce résultat combine plusieurs origines de mesure : il constitue un nuage
cartésien commun, pas un nouveau scan polaire provenant d'un capteur unique.
Pour corriger les poses manuellement avec l'outil existant :

```bash
python viewer.py scans/scan_*.csv --poses resultats/poses_estimees.json
```

Après correction, relancer la fusion automatique recalculerait les poses.
Le fichier de poses du visualiseur et les exports fusionnés ne sont pas
synchronisés automatiquement.

### Modules du projet

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
