# Jour 2 — Localisation entre scans et comparaison commune

Les trois types de référence disposent maintenant d'une méthode et d'une
évaluation numérique. [localisation_scans.py](./localisation_scans.py) ajoute
deux méthodes pour transformer les points d'un scan requête dans le repère
d'un **scan unique de référence**. Un nuage fusionné de plusieurs origines
n'est pas présenté comme un scan de référence unique.

## 1. Méthodes entre scans

### Recherche grossière puis fine

`ScanMatcher` utilise un arbre KD pour calculer la distance au point du scan
de référence le plus proche. Le coût est la moyenne des distances au carré,
bornées à 0,6 m. Le moteur de recherche est partagé avec les méthodes sur
grille et sur segments : pas de 25 cm/5°, puis 5 cm/1°, puis 1 cm/0,2°,
avec huit candidats distincts conservés entre niveaux. La fenêtre par
défaut est ±2 m sur chacun des axes et ±30° autour de la pose initiale.
La recherche ne garantit pas un optimum global hors de cette fenêtre.

### ICP point-à-point

`ICPMatcher` part exactement de la pose initiale fournie, sans recherche
globale cachée. Chaque itération :

1. transforme les points de la requête avec la pose courante ;
2. associe chaque point à son plus proche voisin dans la référence ;
3. exclut les distances supérieures à 1,2 m et conserve les correspondances
   dans les 80 % de distances les plus faibles ;
4. ajuste une rotation et une translation par moindres carrés/SVD, sans
   réflexion, puis compose cette correction avec la pose courante.

Il faut au moins six correspondances. L'algorithme s'arrête après 80
itérations au maximum, ou lorsque la correction est inférieure à 10⁻⁵ m
et 10⁻³ degré. La convergence numérique ne prouve pas que la pose est
physiquement correcte. Contrairement aux trois recherches hiérarchiques,
l'ICP local n'est pas contraint par la fenêtre ±2 m/±30° ; les sorties
signalent s'il en sort. Ce choix et le rejet des correspondances font
partie de la différence entre les méthodes comparées.

## 2. Protocole de comparaison identique

[comparaison_localisation.py](./comparaison_localisation.py) compare :

- `grille` : champ de distance aux cellules occupées, résolution 5 cm ;
- `vectorielle` : distance exacte aux segments finis ;
- `scan_recherche` : distance aux points du scan de référence ;
- `scan_icp` : ICP point-à-point local.

Pour chaque cas, **les quatre méthodes reçoivent les mêmes points de requête
et la même pose initiale**. Leurs références sont toutes dérivées du même
nuage de référence. La pose vraie n'est pas fournie aux algorithmes ; elle
sert à construire le test et à calculer les erreurs après le recalage.

### Simulation avec poses connues

La scène simulée est une pièce de 10 × 8 m avec des obstacles asymétriques.
Chaque scan résulte d'un lancer de 450 rayons à 360° depuis sa propre pose,
en conservant le premier impact visible et en ajoutant un bruit radial
gaussien indépendant d'écart-type **0,005 m**. Les graines sont fixées.
Il ne s'agit pas de déplacer artificiellement une copie exacte du scan
de référence : les requêtes voient la scène depuis des positions distinctes,
avec des différences d'occlusion et d'échantillonnage.

La référence est prise en `(0, 0, 0°)`. Les requêtes proviennent des poses
connues `(0,7 ; −0,5 ; 12°)`, `(−0,8 ; 0,6 ; −15°)` et
`(1,1 ; 0,8 ; 20°)`, soit à moins de 1,4 m de la référence. Un point de
requête sur cinq est utilisé (90 impacts), identiquement pour les quatre
méthodes. Les coordonnées des poses sont exactes par construction ; le
bruit de 5 mm porte sur les distances observées et non sur ces poses.

### Points réels réservés

Pour chacun des classeurs Scan 1, Scan 2 et Scan 3, 80 % des points (363)
construisent les références et 20 % (91) restent réservés au test. Ces
derniers sont exprimés dans trois repères virtuels connus, avec les mêmes
trois transformations que la simulation. Les points réservés n'interviennent
pas dans la construction des grilles, l'extraction des segments ou les arbres
KD de référence. Ce protocole ne remplace pas une paire d'acquisitions
physiques prises à des positions mesurées.

Les résultats de ces expériences contrôlées restent séparés de ceux de la
simulation. Ils ne doivent pas être présentés comme une mesure de précision
physique du robot. Ils diffèrent aussi des évaluations antérieures à cinq
transformations : cette comparaison commune utilise trois poses et trois
initialisations par méthode.

### Sensibilité à l'initialisation

Les décalages suivants sont ajoutés à la pose connue pour générer les poses
initiales, identiques pour les quatre méthodes :

| Niveau | Décalage X | Décalage Y | Décalage d'angle | Erreur de position initiale |
| --- | ---: | ---: | ---: | ---: |
| Proche | +0,10 m | −0,10 m | +2° | 0,14 m |
| Moyenne | +0,80 m | −0,60 m | +15° | 1,00 m |
| Difficile | +1,50 m | −1,00 m | +25° | 1,80 m |

Ces initialisations sont construites à partir de la vérité connue pour
faire une expérience contrôlée, pas pour prétendre connaître la pose d'un
robot en exploitation. Toutes les vérités sont dans les fenêtres de recherche
pour les trois niveaux testés. Une initialisation plus éloignée peut faire
échouer les recherches bornées ou conduire l'ICP à un minimum local.

Au total : **4 jeux de données × 3 requêtes × 3 initialisations × 4 méthodes
= 144 essais**. Les CSV de sensibilité conservent les résultats séparés
pour chaque niveau, et aucun échec n'est retiré du dénominateur.

## 3. Mesures et résultats

L'erreur de position est la norme euclidienne entre positions estimée et
connue, en mètres. L'erreur angulaire est le plus petit écart absolu modulo
360°, en degrés. Un essai est réussi si **les deux** erreurs respectent
**0,10 m et 1°**. Le temps mesure le recalage seulement ; les temps de
préparation des références sont enregistrés séparément. Les durées sont
indicatives du poste et de la charge de cette exécution.

### Simulation : trois poses et trois initialisations, neuf essais par méthode

| Méthode | Erreur moyenne de position | Erreur moyenne d'angle | Temps moyen | Réussites |
| --- | ---: | ---: | ---: | ---: |
| Grille | 0,0245 m | 0,533° | 0,211 s | 9/9 |
| Vectorielle | 0,0133 m | 0,267° | 1,900 s | 9/9 |
| Recherche entre scans | 0,0108 m | 0,333° | 2,126 s | 9/9 |
| ICP entre scans | 0,0085 m | 0,277° | 0,021 s | 9/9 |

### Points réels contrôlés : erreurs moyennes de position

| Données | Grille | Vectorielle | Recherche entre scans | ICP entre scans |
| --- | ---: | ---: | ---: | ---: |
| Scan 1 | 0,0251 m | 0,0407 m | 0,0200 m | 0,0158 m |
| Scan 2 | 0,0559 m | 0,0224 m | 0,0518 m | 0,0294 m |
| Scan 3 | 0,0290 m | 0,0114 m | 0,0244 m | 0,0270 m |

Dans ces conditions, les quatre méthodes réussissent les neuf essais de
chaque jeu, soit 100 % dans chaque groupe. L'augmentation de l'erreur
initiale ne change pas les poses finales des recherches hiérarchiques dans
cette plage. L'erreur finale de l'ICP augmente légèrement sur certains scans
réels, tout en restant sous les seuils. Les courbes et les résultats par
niveau sont exportés. **Aucune absence générale de sensibilité ni robustesse
globale n'est démontrée** : les scènes, le bruit et les initialisations
testés restent limités.

L'ICP est le plus rapide sur ces jeux. Les recherches hiérarchiques évaluent
beaucoup plus de poses. Les performances ne peuvent pas être généralisées à
une salle symétrique, des scans sans recouvrement ou un déplacement hors
fenêtre. Les coûts résiduels propres aux différentes représentations ne
sont pas directement équivalents ; ce sont les erreurs de pose connues,
les temps et les taux de réussite qui servent à la comparaison.

## 4. Paire de scans réellement acquis et incertitude

Un recalage supplémentaire utilise Scan 2 comme référence et Scan 3 comme
requête, avec la même pose initiale `(0 ; 0 ; 90°)` pour les deux méthodes
entre scans. Le résultat transforme le repère de Scan 3 vers celui de Scan 2.

| Méthode | X estimé | Y estimé | Angle estimé | RMS résiduel borné | Temps |
| --- | ---: | ---: | ---: | ---: | ---: |
| Recherche | 0,000 m | 0,080 m | 87,600° | 0,056 m | 10,356 s |
| ICP | −0,002 m | 0,069 m | 88,205° | 0,063 m | 0,022 s |

Les deux méthodes atteignent environ 99,8 % de points à moins de 20 cm de
la référence. Cela constitue un diagnostic d'alignement, **pas une erreur
de pose mesurée**. La proximité physique des acquisitions reste à confirmer.
Les 90° de l'initialisation sont une approximation cohérente avec les
rotations signalées, pas une mesure exacte du déplacement entre ces fichiers.

**Incertitudes sur le robot :** aucune position de référence métrique ni
orientation exacte n'a été relevée. Leur incertitude n'est donc pas
quantifiée ; aucune borne « ± x cm / ± y° » n'est inventée. Les champs
d'erreur physique sont `null` dans le JSON. Les anciennes poses issues de
l'ICP de fusion ne sont pas utilisées comme vérité terrain physique.

Pour compléter la validation matérielle, il faudra mesurer une paire de
poses proches dans un repère commun, relever l'incertitude des instruments
et répéter les acquisitions immobiles. On pourra alors distinguer erreur
de recalage, bruit du capteur et incertitude de la référence.

### Toutes les paires des classeurs Scan 1, Scan 2 et Scan 3

[localisation_paires.py](./localisation_paires.py) traite également toutes
les combinaisons des scans fournis. Pour les trois classeurs, cela donne
Scan 2 vers Scan 1, Scan 3 vers Scan 1 et Scan 3 vers Scan 2.

Lorsque leurs poses relatives sont inconnues, un amorçage automatique par
ICP multidépart estime la pose initiale. La recherche et l'ICP de raffinement
reçoivent ensuite **cette même pose**. Le temps et les éventuelles solutions
alternatives de l'amorçage sont conservés. Ce diagnostic dépend donc d'une
initialisation déjà issue de l'ICP et ne remplace pas la comparaison commune
à 144 essais avec initialisations imposées indépendamment.

```bash
python localisation_paires.py "/Users/amazighe/Desktop/Scan 1.xlsx" "/Users/amazighe/Desktop/scan 2.xlsx" "/Users/amazighe/Desktop/scan 3.xlsx"
```

Les résultats sont dans `resultats/recalage_scans_reels.png`, `.csv` et
`.json`, avec une image et un JSON par paire (`_paire_1`, `_paire_2`,
`_paire_3`). Le gris indique la référence, le bleu la requête après recherche
et le vert après ICP. Les fichiers d'origine restent inchangés.

Les raffinements ICP donnent les poses candidates suivantes :

| Transformation vers le repère de référence | X | Y | Angle | Impacts à moins de 20 cm |
| --- | ---: | ---: | ---: | ---: |
| Scan 2 vers Scan 1 | −0,378 m | −2,431 m | 90,692° | 67,0 % |
| Scan 3 vers Scan 1 | −0,461 m | −2,430 m | 178,166° | 67,8 % |
| Scan 3 vers Scan 2 | −0,002 m | 0,070 m | 88,199° | 99,8 % |

Le recouvrement plus faible avec Scan 1 et les poses estimées imposent une
vérification visuelle et physique. La proximité réelle des positions reste
à confirmer ; les déplacements de l'ordre de 2,4 m estimés par rapport à
Scan 1 ne sont pas des mesures de vérité terrain. Les erreurs physiques
restent vides dans le CSV et `null` dans le JSON. Les seuils de réussite
sur poses connues ne sont pas appliqués à ces paires sans référence mesurée.

## 5. Commandes et fichiers

```bash
python comparaison_localisation.py --scans "/Users/amazighe/Desktop/Scan 1.xlsx" "/Users/amazighe/Desktop/scan 2.xlsx" "/Users/amazighe/Desktop/scan 3.xlsx"
python localisation_scans.py "/Users/amazighe/Desktop/scan 2.xlsx" "/Users/amazighe/Desktop/scan 3.xlsx" --initial 0 0 90 --output resultats/localisation_scans_paire_2_3
```

Avec `comparaison_localisation.py` sans `--scans`, seule la simulation est
évaluée. Les deux outils acceptent `--output` pour choisir le préfixe.
`localisation_scans.py` propose `--method search`, `--method icp` ou `both`.

Principales sorties dans `resultats/` :

- `comparaison_localisation.csv` : les 144 essais, poses, erreurs, temps et succès ;
- `comparaison_localisation_resume.csv` et `comparaison_localisation.md` :
  bilan par jeu et méthode ;
- `comparaison_localisation_sensibilite.csv` : bilan par niveau initial ;
- `comparaison_localisation.json` : protocole, scènes, préparations et détails ;
- `comparaison_localisation.png` : courbes de sensibilité ;
- `comparaison_localisation_scans_simulation.png` : superposition des scans simulés ;
- les fichiers `_donnees.npz`, `_grille.npz` et `_segments.csv` : références,
  requêtes et vérités nécessaires pour vérifier le protocole ;
- `localisation_scans_paire_2_3.json` et `.png` : diagnostic de la paire réelle.

Tests :

```bash
python -m pytest tests/test_localisation_scans.py tests/test_comparaison_localisation.py -q
```

Ils vérifient notamment le premier impact visible en simulation, la pose du
capteur, la reproductibilité du bruit, la récupération d'une transformation,
l'absence de réflexion, l'égalité des entrées entre méthodes, le calcul des
taux de réussite et l'absence de vérité terrain inventée sur les fichiers réels.
