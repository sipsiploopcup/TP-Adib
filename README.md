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
