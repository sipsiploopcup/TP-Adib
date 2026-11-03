# TP — Jour 2 : localisation sur carte d'occupation

La première référence traitée est une **grid map**. Une méthode fonctionnelle
est suffisante pour cette partie : les différentes pistes de l'énoncé ne sont
pas toutes obligatoires. La [carte vectorielle](./JOUR2_VECTORIELLE.md) dispose
également d'un module distinct. La [localisation entre scans et la comparaison
commune](./JOUR2_SCANS_COMPARAISON.md) complètent le troisième type de référence.
Le lien Moodle n'a pas pu être consulté.

## Méthode

[localisation_grid.py](./localisation_grid.py) cherche la pose `(x, y, theta)`
d'un scan dans une grille existante au format NPZ. La carte reste inchangée.
Le scan doit recouvrir des obstacles représentés dans cette carte.

1. Construire le champ de distance euclidienne aux cellules occupées, à
   partir des états de [gridmap.py](./gridmap.py).
2. Transformer les points du scan avec chaque pose candidate.
3. Lire les distances par interpolation bilinéaire aux centres des cellules.
4. Minimiser la moyenne des distances au carré, bornées à 0,6 m. Les points
   hors carte reçoivent également cette pénalité maximale ; ils ne sont pas
   supprimés du score.
5. Rechercher d'abord avec des pas de 25 cm et 5°, puis affiner les huit
   candidats spatialement ou angulairement distincts avec des pas de 5 cm/1°,
   puis 1 cm/0,2°. La première grille répartit les poses sur toute la fenêtre
   demandée avec un pas au plus égal au pas grossier annoncé.

Il s'agit d'une recherche hiérarchique sur les **poses**, utilisant un champ
de distance à résolution fixe. Ce n'est pas une pyramide de cartes ni une
recherche exhaustive de toutes les poses à la résolution finale. La fenêtre
par défaut est ±2 m sur chaque axe et ±30° autour de la pose initiale.
La recherche peut manquer la bonne pose si elle est hors fenêtre ou si un
autre alignement possède un meilleur score dans une salle symétrique.

Le résultat donne la pose, le coût RMS par rapport à la carte, le recouvrement
à 20 cm, le temps et le nombre de poses examinées. Un recouvrement inférieur
à 50 % donne le statut `insufficient_overlap`. Sinon, `candidate` signifie
qu'une pose candidate est fournie : ce n'est pas une certitude probabiliste.
`on_search_boundary` indique un optimum situé près des limites de recherche.
Le score utilise les impacts ; il ne vérifie pas un modèle complet des
rayons traversant les espaces libres et inconnus.

## Évaluation chiffrée sans poses physiques mesurées

Le classeur Scan 1 contient 454 impacts valides. Pour éviter d'utiliser les
mêmes impacts comme référence et comme requête, un point sur cinq est réservé :

- **363 points** construisent une carte de référence de 5 cm.
- **91 points** servent au test et ne sont pas intégrés comme impacts dans la carte.
- Ces points de test sont exprimés dans cinq repères virtuels dont les
  transformations sont connues. Leurs distances géométriques sont celles
  d'un changement de repère imposé, pas de nouvelles acquisitions physiques.
- La pose initiale de la recherche reste `(0, 0, 0°)` dans chaque cas ; la
  transformation attendue n'est transmise qu'au calcul des erreurs et au tracé.

Ce dernier point concerne les scans individuels. Pour le classeur Fusion,
la pose initiale est la pose de base estimée de l'acquisition interrogée ;
la perturbation imposée n'est pas donnée à la recherche.

| Cas | Pose imposée (m, m, °) | Pose retrouvée (m, m, °) | Erreur de position | Erreur angulaire |
| --- | --- | --- | --- | --- |
| 1 | (0 ; 0 ; 0) | (0 ; 0,03 ; −0,8) | 3,00 cm | 0,8° |
| 2 | (0,7 ; −0,5 ; 12) | (0,7 ; −0,48 ; 11,2) | 2,00 cm | 0,8° |
| 3 | (−0,8 ; 0,6 ; −15) | (−0,79 ; 0,64 ; −15,8) | 4,12 cm | 0,8° |
| 4 | (1,1 ; 0,8 ; 20) | (1,11 ; 0,81 ; 19,2) | 1,41 cm | 0,8° |
| 5 | (−1,2 ; −0,9 ; −22) | (−1,21 ; −0,86 ; −22,8) | 4,12 cm | 0,8° |

Résultats de cette exécution : erreur moyenne de position **2,93 cm**, maximale
**4,12 cm**, erreur angulaire moyenne **0,8°**, durée moyenne **0,22 s** sur le
poste utilisé. Les cinq cas respectent le critère choisi de 10 cm et 1°.
Les durées varient selon le matériel et la charge.

**Portée de ces chiffres :** récupération de transformations contrôlées sur
une acquisition réelle partagée en deux sous-ensembles. Ce n'est ni une
évaluation sur cinq scans indépendants, ni une mesure de précision physique
du robot. La discrétisation, les points réservés et la fonction de coût
peuvent déplacer le minimum, même pour une transformation connue. Une
validation plus complète demandera un autre scan dans la zone couverte et
une pose mesurée indépendamment. Les anciennes poses ICP ne doivent pas être
présentées comme une vérité terrain physique.

## Exécution

Depuis le projet et son environnement Python :

```bash
python localisation_grid.py evaluate "/Users/amazighe/Desktop/Scan 1.xlsx"
```

Cette commande produit dans `resultats/` :

- `localisation_grid_scan_1.png` : avant/après pour le cas 2 ;
- `localisation_grid_scan_1.csv` : mesures quantitatives des cinq cas ;
- `localisation_grid_scan_1.json` : protocole, poses et résultats détaillés ;
- `localisation_grid_scan_1_reference.npz` : carte issue des 363 points.

Pour localiser un autre scan dans une carte existante :

```bash
python localisation_grid.py match carte_reference.npz nouveau_scan.xlsx \
  --initial 0 0 0 --xy-range 2 --angle-range 30 \
  --output resultats/localisation_nouveau_scan
```

Adapter les chemins, la pose initiale approximative et la fenêtre. Les angles
sont en degrés antihoraires, les positions en mètres. Les scans d'entrée
sont des CSV/XLSX uniques avec `dist_mm`, `x_lib_mm`, `y_lib_mm`. Un classeur
fusionné contenant plusieurs origines n'est pas une requête LiDAR unique.

```bash
python -m pytest tests/test_localisation_grid.py -q
```

Les tests vérifient des poses connues sur une scène asymétrique synthétique,
le passage de ±180°, les centres des cellules, la pénalisation hors carte,
le manque de recouvrement et le rejet d'entrées invalides.

## Extension aux scans 2, 3 et Fusion

```bash
python localisation_grid.py evaluate "/Users/amazighe/Desktop/scan 2.xlsx" --title "Scan 2" --output resultats/localisation_grid_scan_2
python localisation_grid.py evaluate "/Users/amazighe/Desktop/scan 3.xlsx" --title "Scan 3" --output resultats/localisation_grid_scan_3
python localisation_grid.py evaluate "/Users/amazighe/Desktop/scan fusion.xlsx" --title "Fusion" --output resultats/localisation_grid_fusion
```

Chaque préfixe produit une image PNG du cas 2, les résultats CSV/JSON et la
carte NPZ de référence. Les fichiers sources ne sont pas modifiés.

| Données | Points de référence | Erreur moyenne de position | Erreur moyenne d'angle | Réussites ≤10 cm et ≤1° |
| --- | --- | --- | --- | --- |
| Scan 1 | 363 | 2,93 cm | 0,80° | 5/5 |
| Scan 2 | 363 | 5,08 cm | 1,00° | 5/5 |
| Scan 3 | 363 | 2,42 cm | 0,80° | 5/5 |
| Fusion | 4 352 | 52,27 cm | 2,96° | 2/5 |

Ces valeurs décrivent les tests contrôlés, pas une précision physique du
capteur. Les durées moyennes de cette exécution se situent autour de 0,21
à 0,22 seconde par cas sur le poste utilisé.

### Conservation des origines dans Fusion

La carte de référence est construite avec 80 % des points de **chacune des
12 acquisitions**, en traçant les rayons depuis son origine propre. Les
1 092 points restants sont réservés. Chaque cas utilise les 20 % réservés
d'une seule acquisition : cinq acquisitions distinctes sont interrogées,
dans l'ordre du classeur, avec les cinq perturbations annoncées. Chaque
requête contient ici 91 points. Le nuage fusionné entier n'est jamais
présenté comme un scan provenant d'une seule origine.

Les poses de base et la carte fusionnée dépendent du recalage estimé du
Jour 1. Une petite erreur de récupération d'une perturbation ne prouverait
donc pas l'exactitude de ce recalage initial.

### Échecs observés sur Fusion

L'énoncé demande une requête couvrant une zone représentée dans la carte.
Le JSON fournit pour chaque cas le recouvrement des impacts à 20 cm **à la
pose imposée**, calculé uniquement après la recherche pour diagnostiquer le
test. Ce diagnostic ne sert pas à choisir la pose estimée ni à filtrer les
résultats. Un seuil indicatif de 50 % signale les cas de couverture faible ;
il ne constitue pas une preuve de couverture géométrique complète.

| Cas Fusion | Recouvrement à la pose imposée | Erreur de position | Erreur d'angle |
| --- | --- | --- | --- |
| 1 | 15,4 % | 121,20 cm | 2,80° |
| 2 | 61,5 % | 27,07 cm | 0,40° |
| 3 | 70,3 % | 7,62 cm | 0,80° |
| 4 | 24,2 % | 99,40 cm | 10,00° |
| 5 | 100,0 % | 6,08 cm | 0,80° |

Les cas 1 et 4 ne fournissent pas une validation dans de bonnes conditions
de recouvrement. Leur grande erreur ne doit pas être masquée dans la moyenne.
Pour ces cas, la fonction de coût préfère même la pose déplacée à la pose
imposée : le problème ne se limite pas à un pas de recherche trop grossier.
Il faut vérifier l'alignement des acquisitions, les contradictions dans la
grille et la fonction de coût avant de déclarer la localisation sur Fusion
validée. Le cas 2 échoue également au seuil de position malgré un
recouvrement supérieur à 50 %. Les scans individuels sont plus cohérents
dans ce protocole.

Les diagnostics `reference_coverage_ok`, `reference_overlap_20cm` et
`rmse_at_imposed_pose_m` sont spécifiques à cette évaluation contrôlée.
La commande `match` sur un scan inconnu ne connaît pas sa pose réelle et
ne peut pas calculer ces erreurs de référence.
