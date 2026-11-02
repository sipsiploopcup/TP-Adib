# Handoff Codex : TP Lidar LD06, Jour 1

L'utilisateur est étudiant (EIGSI). Il réalise ce TP en Python et avance une étape à la fois. Il faut expliquer chaque choix, car c'est un travail noté. Répondre et rédiger le rapport en français.

## État actuel au 2 novembre 2026

Cet état remplace les notes initiales conservées ci-dessous.

Mise à jour après les dernières acquisitions et la fusion :

- 12 CSV réels sont disponibles. La sortie CSV est de nouveau active sur
  l'ESP32. L'étudiant confirme aussi des déplacements ; les poses ne sont
  donc pas supposées être uniquement des rotations de 90°.
- `merge_scans.py` fusionne les 12 scans par ICP multidépart. Les sorties
  sont dans `resultats/` : 5 444 points valides, image et poses estimées.
  Référence : `scan_20261102_170438_115290.csv`. Les poses restent à valider
  physiquement et les zones non observées ne sont pas inventées.
- Huit tests passent. SciPy a été ajouté aux dépendances.
- L'utilisateur demande maintenant de pousser le projet vers
  https://github.com/sipsiploopcup/TP-Adib.git ; l'ancien dépôt reste configuré.
- Une erreur de décompression PyInstaller a été résolue en libérant environ
  890 Mo d'archives d'installation Arduino en cache. Compilation,
  téléversement et acquisition ont ensuite réussi.

Les points suivants décrivent l'avancement antérieur à cette mise à jour.

- L'acquisition CSV (`acquisition.py`) et le visualiseur (`geometry.py`,
  `viewer.py`) sont implémentés. Cinq tests passent dans `.venv`.
- Trois scans réels sont conservés : 454, 455 et 455 points. Les deux premiers
  sont des essais dont les poses relatives ne sont pas mesurées. Le troisième
  a été choisi comme référence conventionnelle ; plusieurs positions avec
  déplacements mesurés restent à acquérir.
- Le LiDAR et le robot pointent dans la même direction selon l'étudiant.
  La salle est rectangulaire. Ne pas inventer de poses ou d'alignements.
- Le professeur demande Teleplot : le programme fourni avec
  `printScanTeleplot(Serial)` a été remis sur l'ESP32. L'étudiant confirme
  l'affichage en direct avec l'extension VS Code Teleplot 1.1.4.
- Arduino IDE 2.3.8, cœur ESP32 3.3.12, LD06 1.0.0. USB à 115200 bauds,
  UART LiDAR à 230400 bauds. Le téléversement à 921600 a échoué ; 115200 réussit.
- Fermer Teleplot avant toute utilisation du port par Arduino ou Python.
  Le lecteur Python actuel attend du CSV, pas le flux Teleplot.
- La méthode Wi-Fi RobotControl a été préparée, compilée et téléversée avec
  Async TCP 3.5.0 et ESP Async WebServer 3.12.1. Son affichage n'a pas été
  validé ; le programme actif a depuis été remplacé par celui de Teleplot.
- L'utilisateur a explicitement demandé de publier tout le travail et les
  ressources du TP. `docs/` inclut désormais les ZIP ; leurs copies sont
  nettoyées des métadonnées Git/macOS. Les originaux téléchargés sont inchangés.
  Le Git imbriqué de LD06 est sauvegardé localement sous `.git/ld06-original-git`.
- Les documents de construction des cartes grid/vectorielle restent à obtenir.
- Les notes initiales suivantes ne demandent pas de recommencer les étapes
  déjà terminées et leur ancienne attente d'accord pour publier est levée.

## Dépôt
- Local : `/Users/amazighe/Desktop/PR/TP-eigsi`
- GitHub : https://github.com/sipsiploopcup/TP-eigsi (branche `main`, suit `origin/main`)
- Python 3.9.6, environnement `.venv` (`source .venv/bin/activate`)
- Documents originaux du TP : `/Users/amazighe/Downloads/TP` (accessible ; une copie se trouve dans `docs/` du dépôt)

## Consignes du TP (verbatim)

**1. Extraction des données lidar**
- Consultez le dossier TP dédié au LD06.
- Apprenez à récupérer les données brutes du lidar.
- Chaque étudiant doit extraire ses propres jeux de scans réels dans la salle (coordonnées polaires). Ces données seront réutilisées pour la suite.

**2. Visualisation des nuages de points**
- Développez un outil permettant d'afficher vos scans sous forme de points cartésiens. L'outil doit permettre d'afficher plusieurs scans simultanément.
- Les différents scans doivent apparaître dans des couleurs distinctes.
- Vous devez pouvoir modifier la pose d'origine de chaque scan indépendamment, pour les déplacer manuellement sur la carte.

**3. Construction d'une grid map 2D**
- Lisez les étapes décrites dans le document « Construction d'une grid map 2D à partir d'un scan lidar » pour comprendre comment construire une grid map à partir d'un scan et mettre à jour une grid map existante.
- Créez un outil qui transforme un scan + pose en carte d'occupation.
- Créez un outil qui prend une carte d'occupation, un scan + pose et qui doit mettre à jour la carte d'occupation (cellules traversées + cellules d'impact).
- Expliquez clairement votre démarche (choix techniques, hypothèses, difficultés, limites) à chaque étape.

**4. Extraction et manipulation de segments**
- Développez un outil capable de :
  - Transformer un nuage de points en un ensemble de segments.
  - Calculer l'angle entre deux segments
  - Dire si deux segments sont « proches » aussi bien « angulairement » qu'en position
  - Fusionner deux segments « proches »
  - Calculer l'intersection de deux segments
  - Découper un segment traversé par un autre segment en conservant le plus long côté.

Pour ce découpage, exigez une intersection située sur les deux segments et strictement à l'intérieur du segment à couper. Une intersection à une extrémité ne change rien. Les segments colinéaires ne sont pas découpés par cette opération. En cas de longueurs égales à la tolérance près, conservez le côté contenant le premier point du segment. Définissez une tolérance numérique et testez ces cas limites.

**5. Construction d'une carte vectorielle**
- Utilisez les fonctions développées précédemment pour construire une carte vectorielle.
- Expliquez, comme pour la grid map, les étapes suivies d'après le document « Construction d'une carte vectorielle 2D à partir d'un scan lidar ».
- Vous êtes libres d'intégrer cette étape dans l'outil du point 3, ou d'en créer un distinct.

Que cela soit pour la cartographie en grid map ou en vectorielle, vous êtes encouragés à ajouter des propriétés supplémentaires à vos cartes : durée de vie des cellules, filtrages, pondérations, etc. Mais commencez simple puis complexifiez progressivement.

## Ce qui a été fait

### Étape 0 : terminée et poussée (commit `76b0c2c`)
- `README.md` (installation, rôle de chaque fichier), `requirements.txt` (pyserial, numpy, matplotlib, pytest), `.gitignore` (`.venv/`, caches, `.DS_Store`).
- Modules **vides** : `acquisition.py`, `geometry.py`, `viewer.py`, `gridmap.py`, `segments.py`, `vectormap.py`.
- `scans/.gitkeep`, `tests/__init__.py`.
- `RAPPORT.md` : squelette avec une section par étape.
- `.venv` créé ; imports vérifiés (pyserial 3.5, numpy 2.0.2, matplotlib 3.9.4, pytest 8.4.2).

### Dossier `docs/` : en local, NON commité
- Copie de `~/Downloads/TP`, dont on a supprimé les `.zip` (les originaux restent dans Downloads).
- Contenu utile au TP : `Acquisition des données issues du Lidar LD06.md`, `LD06 Communication Protocol.pdf`, `LD06 datasheet.pdf`, `ESP32_LD06_Test.ino`, `maptool.py`, `MapTool.html`, `maptoolProcessing.pde`, `Utiliser les outils du module.md`, `Programmer un ESP32 avec lIDE Arduino.md`, `README.md`, `README (1).md`, `requirements.txt`, ainsi que la bibliothèque `LD06/`.
- Hors sujet (voiture Mecanum, NeoPixel, servo, exemples) : `Examples/`, `Adafruit_NeoPixel/`, `MecanumCar_v2/`, `ESP32MecanumCar.ino`, `ESP32RobotControl.ino`, `RobotControl.*`, `support lidar.st*`, `Mecanum car lidar.jpg`.
- **Problème :** `docs/LD06/.git` est un dépôt git imbriqué. Avant de commiter `docs/`, le supprimer ou ignorer ce dossier.
- **Décision en attente de l'utilisateur :** proposition = ne versionner que les fichiers utiles au TP et ignorer le reste. Ne pas commiter `docs/` sans son accord.

### Points d'attention
- Les deux documents « Construction d'une grid map 2D… » et « Construction d'une carte vectorielle 2D… » **ne sont pas dans le dossier TP**. L'utilisateur doit les fournir avant les étapes 3 et 5.
- Le dossier contient `ESP32_LD06_Test.ino` et `maptool.py` : l'acquisition passe probablement par un **ESP32** (et non par un LD06 branché directement en USB). À confirmer en lisant les documents avant d'écrire `acquisition.py`.
- `CODEX_HANDOFF.md` (ce fichier) n'est pas commité.

## Prochaine étape : Étape 1 (acquisition)
1. Lire `docs/Acquisition des données issues du Lidar LD06.md`, `docs/Utiliser les outils du module.md`, `docs/ESP32_LD06_Test.ino`, `docs/maptool.py` et le PDF du protocole.
2. Déterminer la chaîne d'acquisition (ESP32 ou USB, format des données envoyées, outil fourni).
3. Écrire `acquisition.py`, qui enregistre chaque scan en CSV polaire (`angle_deg, dist_mm, intensity`) dans `scans/`. Vérifier le CRC et le sens de rotation.
4. L'utilisateur fait 4 à 6 scans réels à différentes positions de la salle et note une pose approximative (x, y, θ) pour chacun. **Ne jamais inventer de scans** ; des données synthétiques sont acceptables uniquement pour les tests unitaires, en le signalant clairement.
5. Rédiger la section 1 de `RAPPORT.md`.

## Plan des étapes suivantes (à ajuster selon les documents)
- **2. Viewer** (`geometry.py`, `viewer.py`) : conversion polaire → cartésien, `apply_pose` (R(θ)·p + t), matplotlib avec une couleur par scan, sélection du scan actif et curseurs x/y/θ ; sauvegarde des poses dans `poses.json`.
- **3. Grid map** (`gridmap.py`) : grille en log-odds avec une résolution d'environ 5 cm ; Bresenham du robot à l'impact ; cellules traversées `-= l_free`, impact `+= l_occ`, valeurs bornées ; `scan_to_map` = carte vide + `update_map`.
- **4. Segments** (`segments.py` + `tests/test_segments.py`) : extraction par split-and-merge et ajustement ACP ; angle dans [0, π/2] ; proximité (angle, distance perpendiculaire, recouvrement) ; fusion ; intersection paramétrique (`EPS = 1e-9`) ; découpage selon les règles ci-dessus (`TOL = 1e-6` m). Tests : croisement en X des deux côtés, égalité exacte et à ± 1e-8, T sur une extrémité, segment qui touche l'extrémité du coupeur, colinéaires, parallèles, intersection hors des segments.
- **5. Carte vectorielle** (`vectormap.py`) : extraction → repère monde → association → fusion ou ajout → découpage des segments traversés par les rayons (hypothèse, à confirmer dans le document).
- **6. Rapport :** pour chaque étape, démarche, choix, hypothèses, difficultés, limites, avec des captures d'écran.
