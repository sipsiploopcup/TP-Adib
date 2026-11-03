# Scan fusionné de la chambre

## Rapport du TP n° 1 — Adib OUAMOU

- [Rapport PDF (14 pages)](./rapport/TP1_Adib_OUAMOU.pdf)
- [Sources LaTeX et figures — archive pour Overleaf](./rapport/TP1_Adib_OUAMOU_sources.zip)
- [Fichier LaTeX](./rapport/TP1_Adib_OUAMOU.tex)
- [Instructions de compilation](./rapport/README.txt)

Le rapport présente les cinq étapes du TP, les résultats réels, les tests
et les limites des poses estimées et des méthodes retenues.

## Jour 2 — Localisation sur carte d'occupation

Le [programme](./localisation_grid.py) cherche une pose par recherche
grossière puis fine sur un champ de distance aux cellules occupées.
Le [protocole et les diagnostics](./JOUR2_LOCALISATION.md) précisent les
transformations contrôlées, les points réservés et les limites de l'évaluation.

| Données | Erreur moyenne de position | Erreur moyenne d'angle | Réussites ≤10 cm et ≤1° | Résultats |
| --- | --- | --- | --- | --- |
| Scan 1 | 2,93 cm | 0,80° | 5/5 | [Image](./localisation/localisation_grid_scan_1.png), [CSV](./localisation/localisation_grid_scan_1.csv), [JSON](./localisation/localisation_grid_scan_1.json) |
| Scan 2 | 5,08 cm | 1,00° | 5/5 | [Image](./localisation/localisation_grid_scan_2.png), [CSV](./localisation/localisation_grid_scan_2.csv), [JSON](./localisation/localisation_grid_scan_2.json) |
| Scan 3 | 2,42 cm | 0,80° | 5/5 | [Image](./localisation/localisation_grid_scan_3.png), [CSV](./localisation/localisation_grid_scan_3.csv), [JSON](./localisation/localisation_grid_scan_3.json) |
| Fusion | 52,27 cm | 2,96° | 2/5 | [Image](./localisation/localisation_grid_fusion.png), [CSV](./localisation/localisation_grid_fusion.csv), [JSON](./localisation/localisation_grid_fusion.json) |

Ces résultats concernent des repères virtuels imposés sur des points réels
réservés au test, sans vérité terrain physique mesurée. Fusion reste à
valider : deux requêtes ont une couverture insuffisante à la pose imposée
et une troisième échoue malgré un recouvrement supérieur à 50 %. Les
échecs sont conservés. Les cartes de référence NPZ sont dans
[localisation/](./localisation/). Le rapport du TP n° 1 précède ce travail
du Jour 2 et reste inchangé.

```bash
python -m pip install -r requirements.txt
python localisation_grid.py evaluate "Scan 1.xlsx" --title "Scan 1" --output localisation/localisation_grid_scan_1
python localisation_grid.py evaluate "scan 2.xlsx" --title "Scan 2" --output localisation/localisation_grid_scan_2
python localisation_grid.py evaluate "scan 3.xlsx" --title "Scan 3" --output localisation/localisation_grid_scan_3
python localisation_grid.py evaluate "scan fusion.xlsx" --title "Fusion" --output localisation/localisation_grid_fusion
python -m pytest tests/test_localisation_grid.py -q
```

## Jour 2 — Localisation sur carte vectorielle

Le [programme](./localisation_vectorielle.py) minimise la distance exacte
des points aux segments finis par recherche de pose grossière puis fine.
La [méthode et le protocole](./JOUR2_VECTORIELLE.md) détaillent les paramètres,
les références construites sans les points de test et les limites.

| Données | Erreur moyenne de position | Erreur moyenne d'angle | Réussites ≤10 cm et ≤1° | Résultats |
| --- | --- | --- | --- | --- |
| Scan 1 | 3,52 cm | 0,40° | 5/5 | [Image](./localisation_vectorielle/localisation_vectorielle_scan_1.png), [CSV](./localisation_vectorielle/localisation_vectorielle_scan_1.csv), [JSON](./localisation_vectorielle/localisation_vectorielle_scan_1.json) |
| Scan 2 | 2,24 cm | 0,00° | 5/5 | [Image](./localisation_vectorielle/localisation_vectorielle_scan_2.png), [CSV](./localisation_vectorielle/localisation_vectorielle_scan_2.csv), [JSON](./localisation_vectorielle/localisation_vectorielle_scan_2.json) |
| Scan 3 | 1,33 cm | 0,40° | 5/5 | [Image](./localisation_vectorielle/localisation_vectorielle_scan_3.png), [CSV](./localisation_vectorielle/localisation_vectorielle_scan_3.csv), [JSON](./localisation_vectorielle/localisation_vectorielle_scan_3.json) |
| Fusion | 4,56 cm | 0,40° | 4/5 | [Image](./localisation_vectorielle/localisation_vectorielle_fusion.png), [CSV](./localisation_vectorielle/localisation_vectorielle_fusion.csv), [JSON](./localisation_vectorielle/localisation_vectorielle_fusion.json) |

Les cinq cas par jeu de données sont des transformations contrôlées sur des
points réels réservés, pas une mesure de précision physique. Pour Fusion,
les poses de base restent estimées ; le premier cas échoue avec une erreur
de 11,05 cm et reste dans les statistiques. Les références vectorielles
effectivement utilisées sont exportées avec le suffixe `_reference.csv`
dans [localisation_vectorielle/](./localisation_vectorielle/).

```bash
python localisation_vectorielle.py evaluate "Scan 1.xlsx" --title "Scan 1" --output localisation_vectorielle/localisation_vectorielle_scan_1
python localisation_vectorielle.py evaluate "scan 2.xlsx" --title "Scan 2" --output localisation_vectorielle/localisation_vectorielle_scan_2
python localisation_vectorielle.py evaluate "scan 3.xlsx" --title "Scan 3" --output localisation_vectorielle/localisation_vectorielle_scan_3
python localisation_vectorielle.py evaluate "scan fusion.xlsx" --title Fusion --output localisation_vectorielle/localisation_vectorielle_fusion
python -m pytest tests/test_localisation_vectorielle.py -q
```

## Données et résultats

Ce dépôt contient le résultat de la fusion des 12 scans LiDAR et les trois
classeurs Excel réalisés à partir des acquisitions :

- [Image du nuage fusionné](./chambre_fusionnee.png)
- [CSV fusionné — 5 444 points valides](./chambre_fusionnee.csv)
- [Scan 1 — Excel](./Scan%201.xlsx)
- [Scan 2 — Excel](./scan%202.xlsx)
- [Scan 3 — Excel](./scan%203.xlsx)
- [Scan fusion — Excel](./scan%20fusion.xlsx)

Les coordonnées communes sont dans les colonnes `x_m` et `y_m`, en mètres.
Chaque ligne conserve le nom du scan source et ses mesures locales.
Pour Excel, tracer `y_m` en fonction de `x_m` dans un graphique XY.

Les déplacements et rotations ont été estimés automatiquement par recalage.
L'alignement reste à vérifier visuellement ; les zones non observées n'ont
pas été complétées artificiellement. Les scans bruts sont conservés
séparément sur le poste local.

## Cartes d'occupation 2D

Cases de **5 cm**. Blanc : libre ; noir : occupé ; gris : inconnu ou incertain.
Les flèches rouges représentent les origines et orientations du LiDAR.

| Carte | Image | Grille CSV | Carte réutilisable | Métadonnées |
| --- | --- | --- | --- | --- |
| Scan 1 — 454 impacts | [PNG](./cartes/carte_occupation_2d_scan_1.png) | [CSV](./cartes/carte_occupation_2d_scan_1.csv) | [NPZ](./cartes/carte_occupation_2d_scan_1.npz) | [JSON](./cartes/carte_occupation_2d_scan_1.json) |
| Scan 2 — 454 impacts | [PNG](./cartes/carte_occupation_2d_scan_2.png) | [CSV](./cartes/carte_occupation_2d_scan_2.csv) | [NPZ](./cartes/carte_occupation_2d_scan_2.npz) | [JSON](./cartes/carte_occupation_2d_scan_2.json) |
| Scan 3 — 454 impacts | [PNG](./cartes/carte_occupation_2d_scan_3.png) | [CSV](./cartes/carte_occupation_2d_scan_3.csv) | [NPZ](./cartes/carte_occupation_2d_scan_3.npz) | [JSON](./cartes/carte_occupation_2d_scan_3.json) |
| Fusion — 5 444 impacts, 12 origines | [PNG](./cartes/carte_occupation_2d_fusion.png) | [CSV](./cartes/carte_occupation_2d_fusion.csv) | [NPZ](./cartes/carte_occupation_2d_fusion.npz) | [JSON](./cartes/carte_occupation_2d_fusion.json) |

Les trois cartes individuelles utilisent chacune l'origine conventionnelle
`(0, 0, 0°)` et ne sont donc pas directement superposables. La carte Fusion
utilise les 12 acquisitions contenues dans le classeur fusionné. Les couples
de coordonnées locales et globales permettent de retrouver la transformation
déjà appliquée à chaque acquisition et son origine LiDAR. Ces poses restent
celles d'un alignement estimé, pas des positions mesurées.

Le [programme](./gridmap.py) parcourt les rayons avec Bresenham. Les cases
traversées reçoivent une observation libre, les impacts une observation occupée.
Les observations sont accumulées en log-odds bornés ; un impact est prioritaire
sur les passages libres du même scan. Les distances nulles sont ignorées.
Les zones entre les rayons et derrière les impacts restent inconnues si elles
n'ont pas été observées depuis une autre pose.

Les CSV de grille contiennent `-1` pour inconnu/incertain, `0` pour libre,
`1` pour occupé. La ligne zéro correspond au Y minimal. Résolution, origine,
poses et seuils figurent dans les JSON. Les NPZ conservent les observations
permettant de mettre la carte à jour avec un autre scan.

```bash
python -m pip install -r requirements.txt
python gridmap.py "Scan 1.xlsx" --output cartes/carte_occupation_2d_scan_1 --title "Carte d'occupation 2D — Scan 1"
python gridmap.py "scan 2.xlsx" --output cartes/carte_occupation_2d_scan_2 --title "Carte d'occupation 2D — Scan 2"
python gridmap.py "scan 3.xlsx" --output cartes/carte_occupation_2d_scan_3 --title "Carte d'occupation 2D — Scan 3"
python gridmap.py "scan fusion.xlsx" --output cartes/carte_occupation_2d_fusion --title "Carte d'occupation 2D — Fusion"
```

Ajouter `--no-show` pour exporter sans ouvrir de fenêtre. Pour mettre à jour
une carte avec un nouveau scan local, utiliser `--map-in chemin/carte.npz`
et `--pose X Y THETA`, en mètres et degrés antihoraires. La carte s'agrandit
si nécessaire et conserve sa résolution. Le classeur fusionné fournit déjà
ses transformations : ne pas ajouter de pose manuelle dans ce cas.

```bash
python -m pytest tests/test_gridmap.py tests/test_viewer.py -q
```

## Extraction et manipulation de segments

Segments candidats en bleu, mesures en gris, origines LiDAR en rouge.

| Acquisition | Segments | Image | Coordonnées en mètres | Paramètres |
| --- | --- | --- | --- | --- |
| Scan 1 | 15 | [PNG](./segments/segments_scan_1.png) | [CSV](./segments/segments_scan_1.csv) | [JSON](./segments/segments_scan_1.json) |
| Scan 2 | 8 | [PNG](./segments/segments_scan_2.png) | [CSV](./segments/segments_scan_2.csv) | [JSON](./segments/segments_scan_2.json) |
| Scan 3 | 9 | [PNG](./segments/segments_scan_3.png) | [CSV](./segments/segments_scan_3.csv) | [JSON](./segments/segments_scan_3.json) |
| Fusion (12 acquisitions) | 156 | [PNG](./segments/segments_fusion.png) | [CSV](./segments/segments_fusion.csv) | [JSON](./segments/segments_fusion.json) |

Le [programme](./segments.py) utilise split-and-merge avec ajustement ACP.
Il trie chaque scan par angle, coupe les ruptures de plus de 50 cm et
subdivise les groupes dont l'écart maximal à la droite dépasse 5 cm. Les
voisins sont réunis si leur angle diffère de 10° au maximum et si tous
leurs points respectent encore le seuil de 5 cm. Une rupture initiale
interdit la fusion entre les groupes. Un segment retenu nécessite au moins
6 points et 25 cm. Les seuils sont des choix réglables.

Pour Fusion, l'extraction se fait séparément dans chaque scan local, puis
les segments sont transformés avec les poses estimées du classeur. Les
156 segments ne sont pas consolidés entre acquisitions : plusieurs peuvent
représenter le même obstacle. Le programme ne classe pas murs et mobilier
et ne complète pas les zones masquées. Cette superposition précède la
construction d'une carte vectorielle consolidée.

Les CSV contiennent `x1,y1,x2,y2` en mètres. Les JSON conservent les seuils,
les poses, les sources et le nombre de segments par acquisition.

```bash
python segments.py "Scan 1.xlsx" --output segments/segments_scan_1 --title "Extraction de segments — Scan 1"
python segments.py "scan 2.xlsx" --output segments/segments_scan_2 --title "Extraction de segments — Scan 2"
python segments.py "scan 3.xlsx" --output segments/segments_scan_3 --title "Extraction de segments — Scan 3"
python segments.py "scan fusion.xlsx" --output segments/segments_fusion --title "Extraction de segments — Fusion"
```

Ajouter `--no-show` pour exporter sans fenêtre. Les options `--max-error`,
`--max-gap`, `--min-points` et `--min-length` règlent les seuils.

Le module calcule aussi l'angle non orienté (0 à 90°), la proximité en
angle et en distance entre segments finis, la fusion ACP, l'intersection
et le découpage. Le découpage exige une intersection située sur les deux
segments et intérieure au segment à couper. Une intersection à une
extrémité ou des segments colinéaires le laissent intact. Le plus long
côté est gardé ; en cas d'égalité à 1 micromètre près, le côté du premier
point est retenu.

```bash
python -m pytest tests/test_segments.py -q
```

## Cartes vectorielles

Le [programme](./vectormap.py) consolide les segments compatibles après
leur placement dans un repère commun. Les mesures sont grises, les segments
bleus, les intersections orange et les origines LiDAR rouges.

| Carte | Segments avant → après | Image | Coordonnées en mètres | Paramètres et intersections |
| --- | --- | --- | --- | --- |
| Scan 1 | 15 → 15 | [PNG](./cartes_vectorielles/carte_vectorielle_scan_1.png) | [CSV](./cartes_vectorielles/carte_vectorielle_scan_1.csv) | [JSON](./cartes_vectorielles/carte_vectorielle_scan_1.json) |
| Scan 2 | 8 → 7 | [PNG](./cartes_vectorielles/carte_vectorielle_scan_2.png) | [CSV](./cartes_vectorielles/carte_vectorielle_scan_2.csv) | [JSON](./cartes_vectorielles/carte_vectorielle_scan_2.json) |
| Scan 3 | 9 → 9 | [PNG](./cartes_vectorielles/carte_vectorielle_scan_3.png) | [CSV](./cartes_vectorielles/carte_vectorielle_scan_3.csv) | [JSON](./cartes_vectorielles/carte_vectorielle_scan_3.json) |
| Fusion | 156 → 91 | [PNG](./cartes_vectorielles/carte_vectorielle_fusion.png) | [CSV](./cartes_vectorielles/carte_vectorielle_fusion.csv) | [JSON](./cartes_vectorielles/carte_vectorielle_fusion.json) |

Le document détaillé de construction du TP n'est pas fourni. La méthode
proposée utilise l'extraction locale précédente, les poses du classeur et
une association gloutonne suivie d'un ajustement ACP. Une fusion exige un
angle de 8° au maximum, une distance entre segments finis de 20 cm au
maximum et un écart de chaque extrémité à l'autre droite de 8 cm au maximum.
L'ACP commune conserve toutes les extrémités d'origine à poids égal ; leur
écart maximal à la droite ajustée doit rester inférieur ou égal à 8 cm.
Les options `--angle`, `--distance` et `--gap` règlent ces seuils.

Les lacunes de 20 cm ou moins peuvent être reliées. Les ouvertures plus
grandes restent séparées. Les cartes individuelles ont chacune leur repère
local ; Fusion utilise les 12 poses estimées du classeur. Des doublons
subsistent et les obstacles ne sont pas classifiés en murs ou mobilier.

```bash
python vectormap.py "Scan 1.xlsx" --output cartes_vectorielles/carte_vectorielle_scan_1 --title "Carte vectorielle — Scan 1"
python vectormap.py "scan 2.xlsx" --output cartes_vectorielles/carte_vectorielle_scan_2 --title "Carte vectorielle — Scan 2"
python vectormap.py "scan 3.xlsx" --output cartes_vectorielles/carte_vectorielle_scan_3 --title "Carte vectorielle — Scan 3"
python vectormap.py "scan fusion.xlsx" --output cartes_vectorielles/carte_vectorielle_fusion --title "Carte vectorielle — Fusion"
```

Ajouter `--no-show` pour exporter sans fenêtre. Pour un scan seul,
`--pose X Y THETA` choisit la pose (mètres, mètres, degrés antihoraires).
`VectorMap.update(points, pose)` permet d'ajouter un scan en mémoire. La
CLI consolide tous les segments en une fois dans un ordre déterministe ;
des mises à jour successives peuvent produire un regroupement différent.

Les intersections sont recensées dans le JSON avec les numéros des segments
du CSV, à partir de 1. Aucune branche n'est supprimée par défaut. L'option
`--trim-crossings` applique le découpage de l'exercice : chaque segment est
coupé par les autres segments d'entrée dans l'ordre de sortie, en gardant
son plus long côté. En cas d'égalité à 1 micromètre près, le côté du premier
point est retenu ; les extrémités sont ordonnées lexicographiquement. Une
intersection à une extrémité ou des segments colinéaires ne changent rien.
Ce mode est optionnel car un croisement ne prouve pas qu'une branche est
erronée. Les quatre cartes publiées utilisent le mode sans découpage.

Les CSV contiennent `x1,y1,x2,y2` en mètres. Les JSON conservent sources,
poses, seuils, effectifs et intersections. Les classeurs restent inchangés.

```bash
python -m pytest tests/test_vectormap.py -q
```

## Visualiseur interactif

Le [visualiseur](./viewer.py) permet d'afficher plusieurs scans d'acquisition,
avec une couleur et une pose indépendante pour chacun. Le cadrage conserve
la même échelle sur X/Y et centre le nuage dans la zone de dessin.
« Réinitialiser pose » restaure la pose d'ouverture du scan sélectionné ;
« Recentrer » ajuste la vue ; « Enregistrer poses » sauvegarde les poses.

```bash
python -m pip install -r requirements.txt
python viewer.py chemin/vers/scan1.csv chemin/vers/scan2.csv
```

Utiliser les CSV d'acquisition locaux avec les colonnes `dist_mm`, `x_lib_mm`
et `y_lib_mm`. Le CSV fusionné contient aussi ces colonnes locales, mais le
visualiseur ne lit pas ses coordonnées globales `x_m`/`y_m` ; pour afficher
directement la fusion, utiliser l'image ou tracer ces colonnes dans Excel.
Les scans bruts ne sont pas ajoutés à ce dépôt.

Test de la sélection, de la réinitialisation et du centrage :

```bash
python -m pytest tests/test_viewer.py -q
```
