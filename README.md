# Scan fusionné de la chambre

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
pas été complétées artificiellement. Le code et les scans bruts sont conservés
séparément sur le poste local.

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
