# TP — Jour 2 : localisation sur carte vectorielle

## Méthode choisie

[localisation_vectorielle.py](./localisation_vectorielle.py) réalise une
recherche de pose **grossière puis fine**, avec une distance analytique aux
segments de la carte. Une méthode fonctionnelle suffit pour cette partie :
il n'est pas nécessaire d'implémenter toutes les pistes de l'énoncé.

Le scan reste un nuage de points locaux ; la référence est un ensemble de
segments finis en mètres. Il n'y a ni conversion de cette référence en grille,
ni échantillonnage de ses segments en un nuage de référence.

Pour une pose candidate `(x, y, theta)`, chaque point est transformé dans le
repère de la carte. Pour un point `q` et un segment `[a,b]`, on calcule :

```text
u = b - a
t = clip(dot(q-a, u) / dot(u, u), 0, 1)
d(q, [a,b]) = norm(q - (a + t*u))
```

Le coefficient borné `t` impose une projection sur le **segment fini**, et
non sa droite infinie. Le score d'une pose est la moyenne des carrés des
distances au segment le plus proche, chaque distance étant bornée à 0,6 m.
Tous les segments sont examinés, par blocs pour limiter la mémoire.

La classe `VectorMatcher` réutilise les méthodes de recherche de `GridMatcher`
mais remplace complètement la fonction de distance. Les pas de recherche
sont identiques pour comparer les méthodes : 25 cm/5°, puis 5 cm/1°, puis
1 cm/0,2°, avec huit candidats distincts conservés entre niveaux. La fenêtre
par défaut est ±2 m sur chaque axe et ±30° autour de la pose initiale.
Il s'agit d'une recherche hiérarchique dans l'espace des poses, pas d'une
pyramide de cartes ni d'une recherche globale garantie.

Les sorties donnent pose, coût RMS, recouvrement à 20 cm, nombre de poses
évaluées, temps et proximité des limites de recherche. Le statut `candidate`
exige au moins 50 % d'impacts à moins de 20 cm des segments, mais ne constitue
pas une certitude de localisation. Sinon, `insufficient_overlap` signale
un recouvrement insuffisant pour ce critère.

## Évaluation contrôlée

Le découpage des points et les cinq perturbations sont identiques à ceux de
la [localisation sur grille](./JOUR2_LOCALISATION.md) : 80 % des points de
chaque acquisition construisent la référence ; 20 % sont réservés au test.
**L'extraction et la consolidation vectorielles ne voient que les 80 %.**
Les cartes publiées au Jour 1, calculées avec tous les points, ne servent
donc pas de références à cette évaluation.

Pour Fusion, chaque origine est conservée : les segments extraits localement
sont transformés avec les poses estimées du classeur. Une seule acquisition
fournit les points d'une requête ; les cinq cas utilisent les cinq premières
acquisitions du classeur. La recherche reçoit la pose de base comme
initialisation, sans la perturbation à retrouver.

Les erreurs de position et d'orientation sont calculées par rapport aux
transformations imposées. Les diagnostics de couverture à la pose imposée
sont ajoutés **après** la recherche. Les cas difficiles ne sont pas exclus.
Le temps annoncé concerne uniquement la recherche de pose, sans la lecture,
l'extraction ni la consolidation de la référence.

Ce protocole teste la récupération de transformations connues sur des points
réels ; il ne mesure pas la précision physique du robot. Pour les scans
individuels, les cinq cas proviennent du même scan. Pour Fusion, la carte
et les poses de base restent liées au recalage estimé du Jour 1 : un bon
résultat ne valide pas physiquement ce recalage. Une carte vectorielle peut
conserver des segments dédoublés et néanmoins bien expliquer ces requêtes.

## Résultats chiffrés

| Données | Segments de référence (80 %) | Erreur moyenne de position | Erreur moyenne d'angle | Temps moyen de recherche | Réussites ≤10 cm et ≤1° |
| --- | --- | --- | --- | --- | --- |
| Scan 1 | 12 | 3,52 cm | 0,40° | 2,47 s | 5/5 |
| Scan 2 | 5 | 2,24 cm | 0,00° | 0,99 s | 5/5 |
| Scan 3 | 6 | 1,33 cm | 0,40° | 1,14 s | 5/5 |
| Fusion | 84 | 4,56 cm | 0,40° | 14,74 s | 4/5 |

Les effectifs de segments diffèrent des cartes du Jour 1, puisque la
référence est reconstruite sans les points réservés. Les durées sont celles
de cette exécution sur le poste utilisé ; elles dépendent de la charge et
du nombre de segments. L'examen géométrique de tous les segments est ici
plus coûteux que la lecture du champ de distance de la grid map.

Pour Fusion, les erreurs de position des cinq cas sont respectivement
11,05 cm, 2,24 cm, 2,00 cm, 1,41 cm et 6,08 cm. Le premier cas dépasse donc
le seuil de réussite et reste dans les statistiques. Les cinq requêtes ont
un recouvrement supérieur à 50 % à leur pose imposée (79,1 % à 98,9 %).

Par comparaison, l'erreur moyenne sur grille était de 52,27 cm pour Fusion,
avec 2 réussites sur 5. La référence vectorielle conserve des surfaces qui
peuvent devenir incertaines ou libres après les mises à jour contradictoires
de la grille. Elle peut aussi conserver des doublons issus du recalage.
Cette différence de représentation est une explication possible de
l'amélioration ; elle ne démontre pas une supériorité générale et ne prouve
pas que la carte fusionnée correspond exactement à la salle.

## Commandes

```bash
python localisation_vectorielle.py evaluate "/Users/amazighe/Desktop/Scan 1.xlsx" --title "Scan 1" --output resultats/localisation_vectorielle_scan_1
python localisation_vectorielle.py evaluate "/Users/amazighe/Desktop/scan 2.xlsx" --title "Scan 2" --output resultats/localisation_vectorielle_scan_2
python localisation_vectorielle.py evaluate "/Users/amazighe/Desktop/scan 3.xlsx" --title "Scan 3" --output resultats/localisation_vectorielle_scan_3
python localisation_vectorielle.py evaluate "/Users/amazighe/Desktop/scan fusion.xlsx" --title Fusion --output resultats/localisation_vectorielle_fusion
```

Pour chaque jeu de données, le préfixe exporte une image PNG du cas 2, un CSV
de mesures et un JSON détaillé. Le suffixe `_reference.csv` contient les
segments de référence effectivement utilisés, avec `x1,y1,x2,y2` en mètres.

La commande suivante localise un scan dans une carte vectorielle existante :

```bash
python localisation_vectorielle.py match carte_vectorielle.csv nouveau_scan.xlsx \
  --initial 0 0 0 --xy-range 2 --angle-range 30 \
  --output resultats/localisation_vectorielle_nouveau_scan
```

Adapter les chemins et la pose initiale à l'acquisition. Un CSV de carte du
Jour 1 est accepté pour cette commande, à condition que le scan couvre sa
zone ; ce n'est pas le protocole de test avec points réservés. Une mauvaise
initialisation, des obstacles non représentés, des doublons ou des symétries
peuvent conduire à une mauvaise pose. Les objets courbes ou les petits groupes
écartés par l'extraction sont moins bien représentés par cette référence.

## Vérification logicielle

```bash
python -m pytest tests/test_localisation_vectorielle.py -q
```

Les tests couvrent les distances aux segments finis et à leurs extrémités,
le plus proche parmi plusieurs segments, les poses connues sur une géométrie
asymétrique, les cartes invalides, la lecture/écriture CSV, la séparation
référence/test et les exports d'évaluation.
