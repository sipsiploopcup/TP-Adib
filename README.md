# TP Lidar LD06 – Jour 1

Extraction, visualisation et cartographie (grid map et carte vectorielle) à partir de scans LD06.

## Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Structure

| Fichier | Rôle |
|---|---|
| `acquisition.py` | Lecture du LD06 sur le port série, enregistrement des scans en CSV (polaire) |
| `scans/` | Scans réels acquis dans la salle |
| `geometry.py` | Conversion polaire → cartésien, application d'une pose |
| `viewer.py` | Affichage de plusieurs scans, réglage manuel de la pose de chacun |
| `gridmap.py` | Création et mise à jour d'une carte d'occupation |
| `segments.py` | Extraction de segments et opérations géométriques |
| `vectormap.py` | Construction de la carte vectorielle |
| `tests/` | Tests unitaires (`pytest`) |
| `RAPPORT.md` | Démarche, choix techniques, hypothèses, difficultés, limites |
