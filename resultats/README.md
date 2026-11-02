# Fusion des scans de la chambre

- [Nuage fusionné CSV](./chambre_fusionnee.csv) : 5 444 points valides issus
  des 12 CSV d'acquisition. Chaque ligne conserve le fichier source, le numéro
  du point, ses mesures locales et ses coordonnées dans le repère commun.
- [Image de la fusion](./chambre_fusionnee.png) : une couleur par scan et
  une même échelle sur les deux axes, exprimés en mètres.
- [Alignements et diagnostics](./alignements.json) : poses estimées, score,
  recouvrement et éventuelle solution alternative pour chaque scan.
- [Poses pour le visualiseur](./poses_estimees.json) : positions et orientations
  estimées, avec chemins locaux vers les fichiers d'origine.

Le repère est celui du scan `scan_20261102_170438_115290.csv`, fixé par
convention à `(0 m, 0 m, 0°)`. Les rotations sont positives dans le sens
antihoraire. L'étudiant indique avoir aussi déplacé le robot ; les poses
relatives exactes ne sont pas disponibles et n'ont pas été inventées.

L'alignement est une estimation automatique par ICP avec plusieurs points
de départ. Les scans difficiles sont ensuite comparés au nuage déjà aligné.
Le recouvrement géométrique à 20 cm est compris entre 58 % et 100 %.
Ces indicateurs ne prouvent pas l'exactitude des poses : les murs parallèles,
les symétries, les personnes et les objets déplacés peuvent créer des erreurs.
La correspondance avec la salle reste à vérifier par l'étudiant.

Seules les mesures de distance nulle sont retirées de l'export. Les points
à moins de 60 cm du capteur ne participent pas à l'estimation de pose, pour
limiter l'influence du robot ; ils restent présents dans la fusion.
Les scans sources sont conservés sans modification. Aucun mur ou point
manquant n'est créé. Les parties masquées de la chambre restent inconnues.

Pour Excel : utiliser les colonnes **H (`x_m`)** et **I (`y_m`)** comme
abscisses et ordonnées d'une seule série de nuage de points. Les colonnes
polaires sont locales à chaque acquisition, pas relatives à une origine unique.

Reproduction depuis la racine du projet :

```bash
source .venv/bin/activate
pip install -r requirements.txt
python merge_scans.py scans/scan_*.csv --reference scans/scan_20261102_170438_115290.csv --output resultats
```

Validation : huit tests passent, dont trois tests synthétiques de fusion
(transformation rigide, rotation et translation retrouvées, symétrie d'un
rectangle signalée). Les 5 444 coordonnées exportées ont été comparées aux
transformations de leurs points sources, avec une tolérance de 1 micromètre
correspondant à l'arrondi à six décimales du CSV.
