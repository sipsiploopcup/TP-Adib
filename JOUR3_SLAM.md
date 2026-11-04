# TP – Jour 3 : intégration SLAM

Le programme [slam_tp3.py](./slam_tp3.py) relie l’acquisition du TP1,
la localisation entre scans du TP2 et les cartes d’occupation/vectorielles.
Il fonctionne sur fichiers, en simulation et sur un flux de scans en direct.
La navigation automatique réelle exige le robot, une calibration des vitesses
et le firmware Wi-Fi adapté. Elle n’est pas activée par défaut.

## 1. Vérifier le système sans déplacer le robot

Depuis le dossier du projet :

```bash
source .venv/bin/activate
python slam_tp3.py --mode simulate --steps 160 --show --output slam/simulation
```

La fenêtre actualise la carte pendant l’exploration. Fermer la fenêtre ou
appuyer sur Ctrl+C interrompt le traitement et conserve les résultats partiels.
Le robot simulé choisit des frontières entre espace libre et espace inconnu,
s’oriente puis avance par petits pas. Il utilise seulement les scans et la
carte construite, jamais la scène vraie pour choisir son trajet.
La scène vraie sert exclusivement à simuler les impacts, empêcher les collisions
physiques et calculer les erreurs après traitement. Le moteur reçoit une
prédiction de mouvement volontairement biaisée (+3 % en translation, −3 % en
rotation), pas la pose vraie.

## 2. Reconstruire la salle avec les scans déjà acquis

Les 12 fichiers ne constituent pas une acquisition continue : il existe des
interruptions, des rotations importantes et des objets/personnes différents.
Le rejeu chronologique depuis le premier essai est donc un mauvais test de suivi
à petits déplacements. Pour reconstruire la salle, on choisit explicitement
la même référence que lors de la fusion du TP1. Dans ce dépôt, les 12 scans
bruts se trouvent dans `upgrade code with IA/donnees/scans/` :

```bash
D="upgrade code with IA/donnees/scans"
python slam_tp3.py --mode replay "$D"/*.csv \
  --anchor "$D/scan_20261102_170438_115290.csv" \
  --initial-poses resultats/poses_estimees.json \
  --show --output slam/salle
```

Les poses du TP1 initialisent le recalage ; elles ne sont pas imposées comme
résultat final et ne sont pas une vérité terrain indépendante. Le moteur
relocalise chaque scan sur les seules images clés déjà acceptées.
Le rejeu ainsi réordonné n’est pas présenté comme la trajectoire temporelle
physique du robot.

Si `resultats/poses_estimees.json` n’existe pas sur un nouveau poste, le
régénérer avec l’outil du TP1 :

```bash
python merge_scans.py "$D"/*.csv --reference "$D/scan_20261102_170438_115290.csv" --output resultats
```

Sans poses antérieures, on peut tester une recherche sur plusieurs orientations :

```bash
python slam_tp3.py --mode replay "$D"/*.csv \
  --anchor "$D/scan_20261102_170438_115290.csv" \
  --wide-search --output resultats/slam_salle_sans_priors
```

Une pièce symétrique peut fournir plusieurs solutions presque équivalentes.
Le programme rejette alors le recalage plutôt que choisir arbitrairement.
Ne pas diminuer les seuils simplement pour accepter tous les fichiers.

## 3. Localisation et carte en direct par USB

1. Brancher l’ESP32 et alimenter le LiDAR.
2. Utiliser la sortie **CSV**, pas Teleplot, dans le programme embarqué.
3. Fermer le Serial Monitor Arduino et Teleplot pour libérer le port.
4. Repérer le port puis lancer :

```bash
python acquisition.py --list-ports
python slam_tp3.py --mode serial --port /dev/cu.usbserial-1420 \
  --steps 300 --show --output resultats/slam_direct
```

Adapter le port à celui affiché. Ce mode ne commande aucun moteur.
Pour tester le suivi, déplacer lentement le robot entre acquisitions et limiter
le mouvement pendant une révolution du LiDAR. Après 15 secondes sans scan CSV
complet, le programme s’arrête et exporte les données déjà reçues.
Les CSV originaux reçus sont conservés dans `scans_bruts/` du dossier de résultat.

Lors de l’essai du 4 novembre, le port USB était détecté mais aucun octet
n’a été reçu pendant le contrôle et aucun scan CSV complet n’a été reçu
dans le délai de 15 s. L’acquisition réelle en direct reste donc à valider :
vérifier l’alimentation du LiDAR, le programme actif et la sortie CSV.
Aucune commande moteur n’a été envoyée pendant cet essai.

## 4. Essayer le déplacement automatique sur le robot

Le sketch [Robot_SLAM_WiFi.ino](./arduino/Robot_SLAM_WiFi/Robot_SLAM_WiFi.ino)
est prêt à ouvrir dans Arduino IDE. Son [guide de connexion](./arduino/Robot_SLAM_WiFi/README.md)
précise le câblage repris du projet, les bibliothèques et les essais manuels.
Il conserve les routes de commande existantes et fournit `/scan_slam`. Cette réponse
fournit un numéro de scan et des horodatages : une réponse HTTP identique ou trop
ancienne ne peut plus être utilisée comme une nouvelle observation.

1. Ouvrir le sketch, choisir **ESP32 Dev Module**, le port du robot et
   **Upload Speed → 115200**, puis **Upload**. Bibliothèques :
   LD06, MecanumCar_v2, AsyncTCP et ESPAsyncWebServer.
2. Connecter le Mac au Wi-Fi **RobotESP32**, mot de passe de démonstration
   **12345678**. Ouvrir **http://192.168.4.1** et vérifier « LiDAR actif ».
   Le robot reste immobile au démarrage ; la page propose des impulsions
   manuelles de 100 ms et un bouton STOP.
3. Garder les moteurs immobiles et vérifier d’abord la réception :

```bash
python slam_tp3.py --mode robot --robot-url http://192.168.4.1 \
  --steps 20 --show --output resultats/slam_wifi_immobile
```

4. Mesurer la vitesse avant, en m/s, et la vitesse de rotation, en degrés par
   seconde active. Vérifier aussi les sens avant/gauche et le rayon englobant
   du robot. Le rayon de 0,18 m fourni par défaut est à adapter au matériel.
5. Dans une zone d’essai dégagée, remplacer les deux valeurs ci-dessous par
   les vitesses effectivement calibrées :

```bash
python slam_tp3.py --mode robot --robot-url http://192.168.4.1 \
  --auto --speed VITESSE_M_S --turn-rate ROTATION_DEG_S \
  --robot-radius 0.18 --steps 100 --show --output resultats/slam_robot
```

`VITESSE_M_S` et `ROTATION_DEG_S` sont des emplacements à remplacer, pas des valeurs
à copier telles quelles. Aucune vitesse physique n’est déduite des photos.
Le programme refuse `--auto` sans calibration renseignée.

La boucle effectue : scan frais → localisation → mise à jour → choix d’un trajet
libre → impulsion moteur → arrêt → attente de 0,30 s → scan suivant.
Les impulsions durent au plus 0,25 s. Le firmware arrête lui-même un mouvement
à l’échéance, et le client envoie aussi `/stop` après chaque impulsion et en sortie.
Un scan rejeté, une couverture angulaire trop faible, une perte de communication
ou un obstacle sans dégagement observé entraîne l’arrêt. Ctrl+C déclenche aussi
la tentative d’arrêt. La navigation n’essaie pas de traverser l’espace inconnu.
Deux scans initiaux acceptés sont requis avant la première impulsion.
Le dernier scan du budget est suivi d’un arrêt, pas d’un mouvement non relocalisé.

Le mode Wi-Fi enregistre des CSV locaux normalisés dans `scans_locaux/` et les
paquets binaires `.bin` correspondants. L’intensité des CSV normalisés est à 0
(non reportée) ; les mesures originales avec leur intensité restent dans les
paquets binaires. Ce ne sont pas les CSV USB originaux.

## 5. Chaîne de traitement

1. Filtrage des distances nulles, trop proches (< 0,12 m) et hors plage (> 20 m).
   Échantillonnage spatial à 5 cm pour limiter le biais de densité.
2. Premier scan ancré dans un repère conventionnel. Sans référence extérieure,
   le repère n’est pas une position absolue du bâtiment.
3. ICP local sur une sous-carte de huit images clés voisines au maximum.
   Comme pour la fusion du TP1, les impacts à moins de 0,60 m du capteur ne
   participent pas au recalage ; ils restent dans la carte d’occupation.
4. Validation **avant insertion** : au moins 12 correspondances, au moins 65 %
   des points de recalage à moins de 20 cm de la référence et RMSE de ces seuls
   points ≤ 10 cm. La RMSE conditionnelle n’est pas une erreur de pose mesurée.
   En direct, le changement depuis la dernière pose est limité à 0,70 m / 35°.
   Le rejeu discontinu autorise une fenêtre plus large, explicitement séparée.
5. Mise à jour par [update_map](./gridmap.py), avec rayons libres, impacts
   occupés, log-odds bornés et agrandissement de la grille à 5 cm.
6. Nouvelle image clé après 25 cm ou 12°. Les images clés fournissent la
   sous-carte et un graphe de poses SE(2).
7. Recherche de fermeture de boucle après séparation temporelle et parcours
   ≥ 1,5 m, avec proximité spatiale, recalage dans les deux sens et seuils
   renforcés. Optimisation robuste du graphe, premier nœud fixé, puis
   **reconstruction de la grille** aux poses corrigées. Les pondérations
   des contraintes sont heuristiques, pas des covariances certifiées.
8. À l’export, extraction/consolidation des segments avec les outils du TP1.
   Pour borner le coût, la carte vectorielle utilise au maximum 16 images clés
   réparties sur la séquence ; la grille utilise tous les scans acceptés.

La navigation recherche les frontières atteignables dans les cellules libres,
gonfle les obstacles et l’inconnu par le gabarit circulaire plus une marge, puis
suit la première portion droite du chemin. Elle vérifie le volume balayé par
chaque avance avec le scan courant. Un dégagement local n’est possible que si
le trajet reste observé libre et améliore la distance aux obstacles.

## 6. Résultats et limites

Les fichiers de chaque exécution sont :

- `carte_slam.png` : grille, trajectoire et carte vectorielle ;
- `carte_occupation.npz` : grille rechargeable avec résolution et origine ;
- `carte_occupation.csv` : états −1 inconnu/incertain, 0 libre, 1 occupé ;
- `carte_vectorielle.csv` : segments du plan, sans fermeture architecturale inventée ;
- `trajectoire.csv` : poses des scans acceptés, corrigées après optimisation ;
  vérité simulée et erreurs uniquement en simulation, champs vides sur les données réelles ;
- `bilan.json` : acceptations/rejets, temps, contraintes et mesures d’évaluation.

Simulation de 160 acquisitions : 160 intégrées, erreur moyenne de position
≈ 0,005 m et angulaire ≈ 0,10° ; 96,4 % des surfaces connues de la scène sont
retrouvées à 15 cm près. Le taux de réussite est calculé avec les seuils
15 cm / 2° et vaut 100 % sur cette séquence. Ces valeurs concernent **la simulation**.
Aucune boucle n’a été ajoutée sur cette exploration ; le retour fermé et
l’optimisation sont vérifiés séparément dans les tests.

Rejeu des 12 scans réels avec la référence et les initialisations du TP1 :
8 scans acceptés, 4 rejetés avec les seuils actuels. Il produit une carte de la
salle, mais **pas une couverture complète vérifiée**. Les positions physiques
n’ayant pas été mesurées, on ne donne pas d’erreur de pose réelle en mètres/degrés.
Les zones masquées par les personnes, tables ou armoires restent inconnues.
Pour terminer la carte, il faut acquérir de nouveaux scans en suivant une
trajectoire lente, avec recouvrement, autour de ces zones, puis revenir vers
une zone déjà observée pour tester une fermeture de boucle réelle.

La compilation du firmware ESP32 est vérifiée. Elle ne constitue ni un
élément de calibration, ni une validation du déplacement réel. Les angles,
les déplacements et le gabarit doivent être vérifiés sur le robot avant l’essai.

## 7. Tests

```bash
python -m pytest tests/test_slam.py tests/test_slam_robot.py -q
```

Les tests couvrent les transformations SE(2), le rejet sans pollution de carte,
les ambiguïtés d’orientation, le graphe ancré, la fermeture de boucle, le
maintien des petits impacts dans la grille, l’exploration, la fraîcheur des
scans et le débordement des compteurs, ainsi que l’arrêt lors d’une erreur réseau.
Les transports réseau sont remplacés par des doubles de test : les tests
unitaires n’envoient aucune commande à un robot réel.
