# Scan fusionné de la chambre

Ce dépôt contient le résultat de la fusion des 12 scans LiDAR et les trois
classeurs Excel réalisés à partir des acquisitions :

- [Image du nuage fusionné](./chambre_fusionnee.png)
- [CSV fusionné — 5 444 points valides](./chambre_fusionnee.csv)
- [Scan 1 — Excel](./Scan%201.xlsx)
- [Scan 2 — Excel](./scan%202.xlsx)
- [Scan 3 — Excel](./scan%203.xlsx)

Le CSV conserve uniquement les six colonnes des acquisitions originales :
`point_index,angle_deg,dist_mm,intensity,x_lib_mm,y_lib_mm`.
Les 5 444 points sont conservés sans modification de leurs valeurs locales.
Le nom du scan source, les coordonnées globales et le diagnostic d'alignement
ont été retirés à la demande de l'étudiant.

Les X/Y du CSV sont en millimètres dans le repère de chaque acquisition.
Ils ne permettent donc pas de reproduire directement l'image fusionnée,
qui a été générée auparavant avec les poses estimées dans un repère commun.

Les déplacements et rotations ont été estimés automatiquement par recalage.
L'alignement reste à vérifier visuellement ; les zones non observées n'ont
pas été complétées artificiellement. Le code et les scans bruts sont conservés
séparément sur le poste local.
